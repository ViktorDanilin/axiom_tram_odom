import argparse
import json
import math
import sqlite3
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor

from tram_model import tram_model as model
from tram_model.simple_model import WheelIntegrator
from tram_model.simple_model_ros import SimpleModelNode
from tram_model.train_model_ros import LocalEnu, TramModelNode


class DistanceFollower:

    locked = True
    x = y = yaw = curv = 0.0

    def __init__(self):
        self.route = SimpleNamespace(name="offline")
        self.distance = 0.0

    def advance(self, ds, speed):
        self.distance += ds


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def replay(args, model_kind="tram_model"):

    node_class = SimpleModelNode if model_kind == "simple_model" else TramModelNode
    node = node_class.__new__(node_class)
    node._state = model.State(velocity=0, distance=0, omega=0, motor_torque=0)
    attrs = dict(
        _mass=args.mass, _traction_torque=args.traction, _brake_torque=args.brake,
        _brake_torque_limit=args.brake_limit,
        _gain=args.gain, _model_dt=0.0005, _rk3_h_max=0.001,
        _slip_resync=5.0, _max_gap=3.0,
        _adh_cond=0, _slope=0.0, _wheel_scale=1 / 3.6, _wheel_max_age=0.25,
        _wheel_sigma=0.15, _accel_sigma=0.6, _model_accel=0.0, _notch=0.0,
        _t_first=None, _t_last=None, _position_source="route",
        _wheels={"front": None, "rear": None},
        _route_distance_scale=args.route_scale,
        _follower_route_name="offline", _result_route_name="offline",
        _input_period=None, _wheel_trust=1.0, _trust_t=None,
        _adh_scale=1.0, _slip=False, _slip_mps=0.0,
        _model_slip=0.0, _mu=0.0, _fault="",
    )
    for name, value in attrs.items():
        setattr(node, name, value)
    observer = model.CONFIG["observer"]
    node._disagreement = float(observer["disagreement_limit"])
    node._innovation_limit = float(observer["innovation_limit"])
    node._adapt_tau = float(observer["adaptation_tau_s"])
    node._speed_cap = float(model.CONFIG["vehicle_reference"]["design_speed_mps"]) * 1.25
    node._command_min = int(model.CONFIG["adapter"]["command_min"])
    node._command_max = int(model.CONFIG["adapter"]["command_max"])
    node._ekf = SimpleNamespace(
        predict=lambda *a: None, update_accel=lambda *a: None,
        update_speed=lambda *a: None, snap_pose=lambda *a: None)
    node._follower = DistanceFollower()
    node._result_follower = SimpleNamespace(locked=False)
    node._copy_lock = lambda *a: None
    if model_kind == "simple_model":
        node._integrator = WheelIntegrator(node._wheel_max_age, node._max_gap)
        node._wheel_time_offsets = {}
        node._wheel_last_stamps = {}
        node._ekf.x = np.zeros(6)
        node._ekf.P = np.zeros((6, 6))
        node._ekf.position = (0.0, 0.0)
        node._ekf.yaw = 0.0
        node._try_route_match = lambda *a, **kw: None
    node.get_logger = lambda: SimpleNamespace(warn=print, error=print)
    rows, reference, wheels, positions = [], [], [], []
    node._publish = lambda _, t: rows.append(
        (t, node._state.velocity, node._notch, node._model_accel,
         node._follower.distance))
    stream_origins = {}
    first_stream_time = None
    enu = None

    def align_message(topic, msg):
        nonlocal first_stream_time
        if not args.align_streams or model_kind != "tram_model":
            return msg
        raw_time = stamp(msg)
        if first_stream_time is None:
            first_stream_time = raw_time
        if topic not in stream_origins:
            stream_origins[topic] = raw_time
        t = first_stream_time + raw_time - stream_origins[topic]
        if node._t_last is not None:
            t = max(t, node._t_last)
        sec = math.floor(t)
        nanosec = round((t - sec) * 1e9)
        msg.header.stamp.sec = sec + nanosec // 1_000_000_000
        msg.header.stamp.nanosec = nanosec % 1_000_000_000
        return msg

    uri = Path(args.bag).resolve().as_uri() + "?mode=ro"
    with sqlite3.connect(uri, uri=True) as connection:
        topics = dict(connection.execute("SELECT id, name FROM topics"))
        messages = connection.execute(
            "SELECT topic_id, data FROM messages ORDER BY timestamp")
        for topic_id, raw in messages:
            topic = topics[topic_id]
            if topic == "/localization/kinematic_state" and args.reference == "localization":
                msg = deserialize_message(raw, Odometry)
                reference.append((stamp(msg), msg.twist.twist.linear.x))
                positions.append((stamp(msg), msg.pose.pose.position.x,
                                  msg.pose.pose.position.y))
            elif topic == "/sensing/gnss/master/vel" and args.reference == "gnss":
                msg = deserialize_message(raw, TwistStamped)
                reference.append((stamp(msg), math.hypot(
                    msg.twist.linear.x, msg.twist.linear.y)))
            elif topic == "/sensing/gnss/master/fix" and args.reference == "gnss":
                msg = deserialize_message(raw, NavSatFix)
                if (msg.status.status >= 0 and math.isfinite(msg.latitude)
                        and math.isfinite(msg.longitude)):
                    if enu is None:
                        enu = LocalEnu(msg.latitude, msg.longitude, msg.altitude)
                    x, y, _ = enu.to_enu(msg.latitude, msg.longitude, msg.altitude)
                    positions.append((stamp(msg), x, y))
            elif topic == "/vehicle/driver_position_cmd":
                msg = deserialize_message(raw, DriverControllerCommand)
                node._on_command(align_message(topic, msg))
            elif topic in ("/vehicle/front_bogie_velocity",
                           "/vehicle/rear_bogie_velocity"):
                msg = deserialize_message(raw, VelocitySensor)
                wheels.append((stamp(msg), msg.velocity / 3.6))
                node._on_wheel("front" if "front" in topic else "rear",
                               align_message(topic, msg))
    if not rows or not reference or not wheels or not positions:
        raise ValueError("В bag нужны команды, колёса, скорость и позиция эталона")
    return (np.array(rows), np.array(sorted(reference)),
            np.array(sorted(wheels)), np.array(sorted(positions)))


def fit_torque(rows, wheels, mass):

    t, _, notch, _ = rows[:, :4].T
    tw, vw = wheels.T
    speed = np.interp(t, tw, vw)
    accel = (np.interp(t + 0.3, tw, vw) - np.interp(t - 0.3, tw, vw)) / 0.6
    target = (mass * accel + 0.0147 * mass + 125.83 * speed) * model.R

    mask = (
        (speed > 0.15) & (speed < 6) & (t < (t[0] + t[-1]) / 2)
        & (t - 0.3 >= tw[0]) & (t + 0.3 <= tw[-1]))
    decays = np.exp(-np.maximum(np.diff(t), 0) * model.MOTOR_TAU_INV)
    candidates = []
    for brake_notch_limit in (5, 6, 7, 8, 10, 15):
        features = np.zeros((len(t), 2))
        for i in range(1, len(t)):
            request = np.array([max(notch[i - 1], 0),
                                max(min(notch[i - 1], 0), -brake_notch_limit)])
            features[i] = decays[i - 1] * features[i - 1] + (1 - decays[i - 1]) * request
        x, y = features[mask], target[mask]
        if np.linalg.matrix_rank(x) < 2:
            raise ValueError("Для калибровки нужны участки тяги и торможения")
        coefficients = np.linalg.lstsq(x, y, rcond=None)[0]
        for _ in range(15):
            residual = x @ coefficients - y
            weights = np.sqrt(np.minimum(1, 1000 / np.maximum(abs(residual), 1e-9)))
            coefficients = np.linalg.lstsq(
                x * weights[:, None], y * weights, rcond=None)[0]
        mae = float(np.mean(abs(x @ coefficients - y))) / (mass * model.R)
        candidates.append((mae, brake_notch_limit, coefficients))
    mae, limit, coefficients = min(candidates, key=lambda row: row[0])
    return dict(traction=float(coefficients[0]), brake=float(coefficients[1]),
                max_brake_torque_nm=float(limit * coefficients[1]),
                train_acceleration_mae_mps2=mae, samples=int(mask.sum()))


def calculate_metrics(rows, reference, max_reference_gap_s=0.5):
    t, speed, notch, _ = rows[:, :4].T
    ref = np.interp(t, reference[:, 0], reference[:, 1])
    left = np.clip(np.searchsorted(reference[:, 0], t, side="right") - 1,
                   0, len(reference) - 1)
    right = np.minimum(left + 1, len(reference) - 1)
    valid = ((t >= reference[0, 0]) & (t <= reference[-1, 0])
             & (reference[right, 0] - reference[left, 0] <= max_reference_gap_s))
    moving = ref > 1
    middle = (t[0] + t[-1]) / 2
    metrics = {}
    for name, section in (("moving_over_1mps", moving),
                          ("first_half_over_1mps", moving & (t < middle)),
                          ("second_half_over_1mps", moving & (t >= middle)),
                          ("traction_over_1mps", moving & (notch > 0)),
                          ("braking_over_1mps", moving & (notch < 0)),
                          ("low_speed", (ref > 0.05) & (ref <= 1))):
        error = (speed - ref)[valid & section]
        if not len(error):
            raise ValueError(f"Нет движущихся участков для метрики {name}")
        metrics[name] = dict(
            count=len(error), rmse_mps=float(np.sqrt(np.mean(error**2))),
            bias_mps=float(np.mean(error)),
            p95_abs_mps=float(np.quantile(abs(error), 0.95)))
    return metrics


def reference_distance(positions, min_step_m=1.0):

    if len(positions) < 2:
        raise ValueError("Для оценки положения нужны хотя бы две координаты")
    selected = [0]
    for i in range(1, len(positions) - 1):
        if np.linalg.norm(positions[i, 1:3] - positions[selected[-1], 1:3]) >= min_step_m:
            selected.append(i)
    selected.append(len(positions) - 1)
    points = positions[selected]
    segments = np.linalg.norm(np.diff(points[:, 1:3], axis=0), axis=1)
    distance = np.r_[0.0, np.cumsum(segments)]
    return np.interp(positions[:, 0], points[:, 0], distance)


def calculate_position_metrics(rows, positions, sample_period_s=1.0,
                               max_reference_gap_s=0.5, time_range=None):

    order = np.argsort(rows[:, 0], kind="stable")
    ordered = rows[order]
    times, first = np.unique(ordered[:, 0], return_index=True)
    last = np.r_[first[1:] - 1, len(ordered) - 1]
    model_distance = ordered[last, 4]
    start = max(times[0], positions[0, 0])
    end = min(times[-1], positions[-1, 0])
    if time_range is not None:
        start, end = max(start, time_range[0]), min(end, time_range[1])
    grid = np.arange(math.ceil(start / sample_period_s) * sample_period_s,
                     end, sample_period_s)
    left = np.clip(np.searchsorted(positions[:, 0], grid, side="right") - 1,
                   0, len(positions) - 1)
    right = np.minimum(left + 1, len(positions) - 1)
    valid = positions[right, 0] - positions[left, 0] <= max_reference_gap_s
    grid = grid[valid]
    if len(grid) < 2:
        raise ValueError("Недостаточно общих точек положения и модели")
    path = reference_distance(positions)
    reference_s = np.interp(grid, positions[:, 0], path)
    predicted_s = np.interp(grid, times, model_distance)
    reference_s -= reference_s[0]
    predicted_s -= predicted_s[0]
    error = predicted_s - reference_s
    series = np.column_stack((grid, reference_s, predicted_s, error))
    metrics = dict(
        count=len(grid), rmse_m=float(np.sqrt(np.mean(error**2))),
        bias_m=float(np.mean(error)),
        p95_abs_m=float(np.quantile(abs(error), 0.95)),
        max_abs_m=float(np.max(abs(error))),
        final_error_m=float(error[-1]),
        duration_s=float(grid[-1] - grid[0]),
        reference_distance_m=float(reference_s[-1]),
    )
    return metrics, series


def plot_position_comparison(path, current, other=None, labels=("Модель", "Сравнение")):

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    origin = current[0, 0]
    figure, (distance_ax, error_ax) = plt.subplots(
        2, 1, sharex=True, figsize=(16, 8), layout="constrained")
    distance_ax.plot(current[:, 0] - origin, current[:, 1], color="#343a40",
                     label="Координаты эталона")
    for series, color, label in ((current, "#168a73", labels[0]),
                                 (other, "#d95f58", labels[1])):
        if series is None:
            continue
        distance_ax.plot(series[:, 0] - origin, series[:, 2], color=color, label=label)
        error_ax.plot(series[:, 0] - origin, series[:, 3], color=color, label=label)
    distance_ax.set_ylabel("Пройденный путь, м")
    error_ax.set_ylabel("Ошибка положения, м")
    error_ax.set_xlabel("Время от начала оценки, с")
    error_ax.axhline(0, color="#343a40", linewidth=0.8)
    for axis in (distance_ax, error_ax):
        axis.grid(alpha=0.25)
        axis.legend()
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=170)
    plt.close(figure)


def plot_comparison(path, current, original, reference, bag_name,
                    labels=("До настройки", "После настройки")):

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    origin = min(current[0, 0], original[0, 0], reference[0, 0])
    end = max(current[-1, 0], original[-1, 0], reference[-1, 0]) - origin
    figure, (speed_ax, delta_ax) = plt.subplots(
        2, 1, sharex=True, figsize=(18, 8), layout="constrained")
    speed_ax.plot(reference[:, 0] - origin, reference[:, 1],
                  color="#343a40", linewidth=1.2, label="Локализация")
    for rows, color, label in ((original, "#d95f58", labels[0]),
                               (current, "#168a73", labels[1])):
        t, speed = rows[:, 0], rows[:, 1]
        valid = (t >= reference[0, 0]) & (t <= reference[-1, 0])
        reference_speed = np.interp(t[valid], reference[:, 0], reference[:, 1])
        speed_ax.plot(t - origin, speed, color=color, linewidth=0.9,
                      label=label, alpha=0.85)
        delta_ax.plot(t[valid] - origin, speed[valid] - reference_speed,
                      color=color, linewidth=0.8, label=label, alpha=0.8)
    speed_ax.set_title(f"{bag_name} · скорость · весь bag, без сдвига времени")
    speed_ax.set_ylabel("Скорость, м/с")
    speed_ax.legend(ncol=3)
    delta_ax.axhline(0, color="#343a40", linewidth=0.8)
    delta_ax.set_ylabel("Δv, м/с")
    delta_ax.set_xlabel("Время от первого сообщения, с")
    delta_ax.legend()
    for axis in (speed_ax, delta_ax):
        axis.set_xlim(0, end)
        axis.grid(alpha=0.25)
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=170)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="SQLite rosbag (.db3)")
    parser.add_argument("--mass", type=float, default=model.TRAM_WEIGHT)
    parser.add_argument("--traction", type=float, default=model.K_POS)
    parser.add_argument("--brake", type=float, default=model.K_NEG)
    parser.add_argument("--brake-limit", type=float, default=model.MAX_BRAKE_TORQUE)
    parser.add_argument("--gain", type=float, default=0.5)
    parser.add_argument("--route-scale", type=float, default=0.997,
                        help="Масштаб пути динамического route-режима")
    parser.add_argument("--model", choices=("tram_model", "simple_model", "both"),
                        default="tram_model", help="Модель для оценки")
    parser.add_argument("--reference", choices=("localization", "gnss"),
                        default="localization", help="Эталон скорости")
    parser.add_argument("--align-streams", action="store_true",
                        help="Совместить первые метки команд и тележек")
    parser.add_argument("--output", help="Сохранить временные ряды в .npz")
    parser.add_argument("--plot", help="PNG с полным bag и сравнением моделей")
    parser.add_argument("--position-plot", help="PNG с ошибкой продольного положения от времени")
    parser.add_argument("--metrics", help="Сохранить метрики в JSON")
    parser.add_argument("--compare-original", action="store_true",
                        help="Также прогнать исходные параметры модели")
    parser.add_argument("--fit-torque", action="store_true")
    args = parser.parse_args()
    primary_kind = "simple_model" if args.model == "simple_model" else "tram_model"
    rows, reference, wheels, positions = replay(args, primary_kind)
    simple_rows = None
    if args.model == "both":
        simple_rows, simple_reference, _, simple_positions = replay(args, "simple_model")
        if (not np.array_equal(reference, simple_reference)
                or not np.array_equal(positions, simple_positions)):
            raise RuntimeError("Эталонные временные ряды двух прогонов различаются")
    time_range = None
    if simple_rows is not None:
        time_range = (max(rows[:, 0].min(), simple_rows[:, 0].min()),
                      min(rows[:, 0].max(), simple_rows[:, 0].max()))
    position_metrics, position_series = calculate_position_metrics(
        rows, positions, time_range=time_range)
    report = dict(reference=args.reference, align_streams=args.align_streams,
                  mass_kg=args.mass,
                  traction=args.traction, brake=args.brake,
                  max_brake_torque_nm=args.brake_limit,
                  wheel_gain=args.gain, route_scale=args.route_scale,
                  model=primary_kind, metrics=calculate_metrics(rows, reference),
                  position_method="cumulative_path_from_reference_coordinates",
                  position_metrics=position_metrics)
    simple_position_series = None
    if simple_rows is not None:
        simple_position_metrics, simple_position_series = calculate_position_metrics(
            simple_rows, positions, time_range=time_range)
        report["simple_model"] = dict(
            metrics=calculate_metrics(simple_rows, reference),
            position_metrics=simple_position_metrics)
        shared_rows = simple_rows.copy()
        ordered_rows = rows[np.argsort(rows[:, 0], kind="stable")]
        shared_rows[:, 1] = np.interp(
            shared_rows[:, 0], ordered_rows[:, 0], ordered_rows[:, 1])
        report["tram_model_at_simple_timestamps"] = dict(
            metrics=calculate_metrics(shared_rows, reference))
    if args.fit_torque:
        report["fit_first_half_wheels"] = fit_torque(rows, wheels, args.mass)
    original_rows = None
    if args.compare_original or (args.plot and simple_rows is None
                                 and args.model == "tram_model"):
        original_args = argparse.Namespace(**vars(args))
        original_args.mass = 27500.0
        original_args.traction = 1449.0
        original_args.brake = 1176.0
        original_args.brake_limit = 17640.0
        original_rows, original_reference, _, original_positions = replay(original_args)
        if (not np.array_equal(reference, original_reference)
                or not np.array_equal(positions, original_positions)):
            raise RuntimeError("Эталонные временные ряды двух прогонов различаются")
        original_position_metrics, _ = calculate_position_metrics(
            original_rows, positions, time_range=time_range)
        report["original_model"] = dict(
            mass_kg=original_args.mass,
            traction=original_args.traction,
            brake=original_args.brake,
            max_brake_torque_nm=original_args.brake_limit,
            wheel_gain=original_args.gain,
            metrics=calculate_metrics(original_rows, original_reference),
            position_metrics=original_position_metrics)
    if args.plot:
        comparison = simple_rows if simple_rows is not None else original_rows
        labels = (("Простая модель", "Динамическая модель") if simple_rows is not None
                  else ("До настройки", "После настройки"))
        if comparison is None:
            raise ValueError("Для графика нужны --model both или динамическая модель")
        plot_comparison(args.plot, rows, comparison, reference,
                        Path(args.bag).parent.name, labels)
    if args.position_plot:
        plot_position_comparison(
            args.position_plot, position_series, simple_position_series,
            ("Динамическая модель" if primary_kind == "tram_model" else "Простая модель",
             "Простая модель"))
    print(json.dumps(report, indent=2))
    if args.metrics:
        output = Path(args.metrics)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.output:
        arrays = dict(model=rows, reference=reference, wheels=wheels,
                      reference_position=positions, position_series=position_series)
        if simple_rows is not None:
            arrays["simple_model"] = simple_rows
            arrays["simple_position_series"] = simple_position_series
        if original_rows is not None:
            arrays["original_model"] = original_rows
        np.savez(args.output, **arrays)


if __name__ == "__main__":
    main()
