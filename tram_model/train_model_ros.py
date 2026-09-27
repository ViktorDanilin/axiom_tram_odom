import math
import os
from dataclasses import replace
from os.path import join as pathlib_join

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from tram_vehicle_msgs.msg import DriverControllerCommand, VelocitySensor
from visualization_msgs.msg import Marker, MarkerArray

from tram_model import tram_model as model
from tram_model.ekf import EkfParams, PoseSpeedEkf, wrap_angle
from tram_model.geo import LocalEnu
from tram_model.route_map import (
    FollowerParams, RouteFollower, RouteMap, geodetic_to_json_xy,
)
from tram_model.slip import SlipConfig, SlipDetector


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class TramModelNode(Node):
    def __init__(self) -> None:
        super().__init__("tram_model")

        dataset = model.CONFIG["dataset"]
        adapter = model.CONFIG["adapter"]

        self._rk3_h_max = 0.001
        # параметры
        self.declare_parameter("slip_resync_limit_mps", 5.0)
        self.declare_parameter("max_gap_s", 3.0)
        self.declare_parameter("tram_mass_kg", float(model.TRAM_WEIGHT))
        self.declare_parameter("traction_torque_per_notch_nm", model.K_POS)
        self.declare_parameter("brake_torque_per_notch_nm", model.K_NEG)
        self.declare_parameter("max_brake_torque_nm", model.MAX_BRAKE_TORQUE)
        self.declare_parameter("adh_cond", int(model.ADH_COND))
        self.declare_parameter("track_slope_rad", float(model.TRACK_SLOPE))
        self.declare_parameter("wheel_correction_gain", 0.5)
        self.declare_parameter("wheel_max_age_s", float(adapter["wheel_max_age_s"]))

        self.declare_parameter("wheel_velocity_scale",
                               float(adapter["wheel_velocity_scale_to_mps"]))
        self.declare_parameter("velocity_source", "ekf")
        self.declare_parameter("wheel_speed_sigma_mps", 0.15)
        self.declare_parameter("model_accel_sigma_mps2", 0.6)
        self.declare_parameter("ekf_q_pos", 0.01)
        self.declare_parameter("ekf_q_yaw", 1e-4)
        self.declare_parameter("ekf_q_kappa", 1e-6)
        self.declare_parameter("ekf_q_v", 0.05)
        self.declare_parameter("ekf_q_a", 0.5)
        self.declare_parameter("ekf_tau_kappa_s", 15.0)
        self.declare_parameter("ekf_tau_a_s", 2.0)
        self.declare_parameter("use_gnss_init", True)
        self.declare_parameter("gnss_fusion", True)

        self.declare_parameter("gnss_fusion_max_time_s", 0.0)
        self.declare_parameter("gnss_max_age_s", 0.5)
        self.declare_parameter("gnss_pair_max_dt_s", 0.3)
        self.declare_parameter("gnss_pos_sigma_m", 1.5)

        self.declare_parameter("gnss_speed_sigma_mps", 0.06)
        self.declare_parameter("gnss_yaw_sigma_rad", 0.02)
        self.declare_parameter("gnss_reject_reset_count", 30)
        self.declare_parameter("publish_gnss_reference", True)
        self.declare_parameter("pp_lookahead_min_m", 5.0)
        self.declare_parameter("pp_lookahead_time_s", 1.5)
        self.declare_parameter("pp_kappa_sigma", 0.005)
        self.declare_parameter("pp_min_speed_mps", 0.5)
        json_dir = str(model._find_config_path().parent)
        self.declare_parameter("route_dir", json_dir)
        self.declare_parameter("route_files", ["kinematic_state.json"])
        self.declare_parameter("route_origin_lat", 55.8104031450)
        self.declare_parameter("route_origin_lon", 37.4623050517)
        self.declare_parameter("route_json_x0", 103634.578)
        self.declare_parameter("route_json_y0", 86048.242)
        self.declare_parameter("route_json_angle_rad", -0.021921875)
        self.declare_parameter("route_json_scale", 0.999461)
        self.declare_parameter("route_match_max_lateral_m", 25.0)
        self.declare_parameter("route_lock_max_lateral_m", 8.0)

        self.declare_parameter("route_match_max_heading_deg", 60.0)
        self.declare_parameter("route_match_min_count", 3)
        self.declare_parameter("route_switch_radius_m", 60.0)

        self.declare_parameter("route_distance_scale", 0.997)

        self.declare_parameter("position_source", "ekf")
        self.declare_parameter("path_min_step_m", 1.0)
        self.declare_parameter("path_max_points", 10000)
        self.declare_parameter("path_publish_period_s", 0.5)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("log_period_s", 2.0)

        p = self.get_parameter
        self._slip_resync = p("slip_resync_limit_mps").value
        self._max_gap = p("max_gap_s").value
        self._mass = p("tram_mass_kg").value
        self._traction_torque = p("traction_torque_per_notch_nm").value
        self._brake_torque = p("brake_torque_per_notch_nm").value
        self._brake_torque_limit = p("max_brake_torque_nm").value
        for name, value in (("traction_torque_per_notch_nm", self._traction_torque),
                            ("brake_torque_per_notch_nm", self._brake_torque),
                            ("max_brake_torque_nm", self._brake_torque_limit)):
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} должен быть конечным и положительным")
        self._adh_cond = p("adh_cond").value
        self._slope = p("track_slope_rad").value
        self._gain = p("wheel_correction_gain").value
        self._wheel_max_age = p("wheel_max_age_s").value
        self._wheel_scale = p("wheel_velocity_scale").value
        self._velocity_source = p("velocity_source").value
        if self._velocity_source not in ("ekf", "model"):
            raise ValueError("velocity_source должен быть 'ekf' или 'model'")
        self._wheel_sigma = p("wheel_speed_sigma_mps").value
        self._accel_sigma = p("model_accel_sigma_mps2").value
        self._use_gnss_init = p("use_gnss_init").value
        self._gnss_fusion = p("gnss_fusion").value
        self._gnss_fusion_max_time = p("gnss_fusion_max_time_s").value
        self._gnss_max_age = p("gnss_max_age_s").value
        self._gnss_pair_max_dt = p("gnss_pair_max_dt_s").value
        self._gnss_pos_sigma = p("gnss_pos_sigma_m").value
        self._gnss_speed_sigma = p("gnss_speed_sigma_mps").value
        self._gnss_yaw_sigma = p("gnss_yaw_sigma_rad").value
        self._gnss_reject_reset = p("gnss_reject_reset_count").value
        self._publish_reference = p("publish_gnss_reference").value
        self._pp_lookahead_min = p("pp_lookahead_min_m").value
        self._pp_lookahead_time = p("pp_lookahead_time_s").value
        self._pp_kappa_sigma = p("pp_kappa_sigma").value
        self._pp_min_speed = p("pp_min_speed_mps").value
        self._position_source = p("position_source").value
        if self._position_source not in ("ekf", "route"):
            raise ValueError("position_source должен быть 'ekf' или 'route'")
        self._route_distance_scale = p("route_distance_scale").value
        self._path_min_step = p("path_min_step_m").value
        self._path_max_points = p("path_max_points").value
        self._map_frame = p("map_frame").value
        self._base_frame = p("base_frame").value
        self._log_period = p("log_period_s").value

        # модель
        self._state = model.State(velocity=0.0, distance=0.0, omega=0.0, motor_torque=0.0)
        self._model_accel = 0.0
        self._notch = 0.0
        self._t_first = None
        self._t_last = None
        self._input_period = None
        self._wheels = {"front": None, "rear": None}
        observer = model.CONFIG["observer"]
        self._disagreement = float(observer["disagreement_limit"])
        self._innovation_limit = float(observer["innovation_limit"])
        self._adapt_tau = float(observer["adaptation_tau_s"])
        self._speed_cap = float(model.CONFIG["vehicle_reference"]["design_speed_mps"]) * 1.25
        self._command_min = int(adapter["command_min"])
        self._command_max = int(adapter["command_max"])
        self._wheel_trust = 1.0
        self._trust_t = None
        self._adh_scale = 1.0
        self._slip = False
        self._slip_mps = 0.0
        self._slip_ratio = 0.0
        self._slip_kind = "ok"
        self._wheels_slip = SlipDetector(SlipConfig(
            disagreement=self._disagreement,
            innovation_limit=self._innovation_limit,
            speed_cap=self._speed_cap,
        ))
        self._model_slip = 0.0
        self._mu = 0.0
        self._fault = ""

        # EKF
        self._ekf = PoseSpeedEkf(EkfParams(
            q_pos=p("ekf_q_pos").value, q_yaw=p("ekf_q_yaw").value,
            q_kappa=p("ekf_q_kappa").value, q_v=p("ekf_q_v").value, q_a=p("ekf_q_a").value,
            tau_kappa=p("ekf_tau_kappa_s").value, tau_a=p("ekf_tau_a_s").value))

        # GNSS
        self._gnss_locked = False
        self._enu = None
        self._master = None
        self._rover = None
        self._baseline_yaw = None
        self._heading_from_baseline = False
        self._gnss_track = None
        self._gnss_track_yaw = None
        self._gnss_last_fused_t = None
        self._gnss_rejects = 0
        self._gnss_fused_count = 0
        master_off = dataset["gnss_master_in_base_link_m"]
        rover_off = dataset["gnss_rover_in_base_link_m"]
        self._master_offset = (float(master_off["x"]), float(master_off["y"]),
                               float(master_off["z"]))
        self._rover_offset = (float(rover_off["x"]), float(rover_off["y"]),
                              float(rover_off["z"]))
        dx = self._rover_offset[0] - self._master_offset[0]
        dy = self._rover_offset[1] - self._master_offset[1]
        self._antenna_separation = math.hypot(dx, dy)

        self._baseline_in_body = math.atan2(dy, dx)

        # маршрут
        route_files = [str(pathlib_join(p("route_dir").value, f)) for f in p("route_files").value]
        route_files = [f for f in route_files if os.path.isfile(f)]
        self._route_json_params = dict(
            origin_lat=p("route_origin_lat").value,
            origin_lon=p("route_origin_lon").value,
            x0=p("route_json_x0").value, y0=p("route_json_y0").value,
            angle=p("route_json_angle_rad").value,
            scale=p("route_json_scale").value)
        self._route_map = RouteMap(route_files, **self._route_json_params)
        follower_params = FollowerParams(
            max_lateral_m=p("route_match_max_lateral_m").value,
            lock_max_lateral_m=p("route_lock_max_lateral_m").value,
            max_heading_rad=math.radians(p("route_match_max_heading_deg").value),
            min_match_count=int(p("route_match_min_count").value),
            min_speed_mps=self._pp_min_speed,
            switch_radius_m=p("route_switch_radius_m").value)

        self._follower = RouteFollower(self._route_map, follower_params)
        self._result_follower = RouteFollower(self._route_map, follower_params)
        self._follower_route_name = None
        self._result_route_name = None
        self._gnss_along = None
        self._result_speed = 0.0

        self._path = Path()
        self._path.header.frame_id = self._map_frame
        self._ref_path = Path()
        self._ref_path.header.frame_id = self._map_frame
        self._route_path = Path()
        self._route_path.header.frame_id = self._map_frame

        # публикация
        self._pub_velocity = self.create_publisher(VelocitySensor, "/result/velocity", 10)
        self._pub_diag = self.create_publisher(DiagnosticArray, "/result/diagnostics", 10)
        self._pub_slip = self.create_publisher(Bool, "/result/slip_detected", 10)
        self._pub_position = self.create_publisher(Odometry, "/result/position", 10)
        self._pub_path = self.create_publisher(Path, "/result/path", 1)
        self._pub_ref_path = self.create_publisher(Path, "/reference/path", 1)
        self._pub_route_position = self.create_publisher(Odometry, "/route/position", 10)
        self._pub_route_path = self.create_publisher(Path, "/route/path", 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._pub_route_map = self.create_publisher(MarkerArray, "/route/map", latched)
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._static_tf.sendTransform([
            self._antenna_tf("gnss_master", self._master_offset),
            self._antenna_tf("gnss_rover", self._rover_offset),
        ])
        self.create_timer(p("path_publish_period_s").value, self._publish_paths)

        # подписки
        self.create_subscription(DriverControllerCommand, "/vehicle/driver_position_cmd",
                                 self._on_command, 10)
        self.create_subscription(VelocitySensor, "/vehicle/front_bogie_velocity",
                                 lambda m: self._on_wheel("front", m), 10)
        self.create_subscription(VelocitySensor, "/vehicle/rear_bogie_velocity",
                                 lambda m: self._on_wheel("rear", m), 10)
        if self._use_gnss_init or self._gnss_fusion or self._publish_reference:
            self.create_subscription(NavSatFix, "/sensing/gnss/master/fix",
                                     lambda m: self._on_gnss("master", m), 10)
            self.create_subscription(NavSatFix, "/sensing/gnss/rover/fix",
                                     lambda m: self._on_gnss("rover", m), 10)

        fusion = "нет"
        if self._gnss_fusion:
            fusion = ("всегда" if self._gnss_fusion_max_time <= 0
                      else f"первые {self._gnss_fusion_max_time:.0f} с")
        self.get_logger().info(
            f"tram_model: R={model.R:.3f} м, масса={self._mass:.0f} кг, "
            f"момент/ступень: тяга={self._traction_torque:.0f}, "
            f"тормоз={self._brake_torque:.0f} Н*м, "
            f"предел тормоза={self._brake_torque_limit:.0f} Н*м, "
            f"P_max={model.MAX_POWER:.0f} Вт, шаг = интервал входных топиков, "
            f"масштаб скорости колёс={self._wheel_scale:.4f}, "
            f"скорость из {self._velocity_source}, "
            f"GNSS-выставка={'да' if self._use_gnss_init else 'нет'}, GNSS-слияние={fusion}, "
            f"маршрутов в карте: {len(self._route_map.routes)} "
            f"({', '.join(r.name for r in self._route_map.routes) or '—'}), "
            f"/result/position из {self._position_source}")
        self.get_logger().info(
            f"TF антенн в {self._base_frame}: gnss_master {self._master_offset}, "
            f"gnss_rover {self._rover_offset}, базис {self._antenna_separation:.3f} м")

    def _antenna_tf(self, child: str, xyz: tuple[float, float, float]) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._base_frame
        tf.child_frame_id = child
        tf.transform.translation.x = xyz[0]
        tf.transform.translation.y = xyz[1]
        tf.transform.translation.z = xyz[2]
        tf.transform.rotation.w = 1.0
        return tf

    @staticmethod
    def _lever_to_base(east: float, north: float, up: float, yaw: float,
                       offset: tuple[float, float, float]):

        ox, oy, oz = offset
        c, s = math.cos(yaw), math.sin(yaw)
        return (east - (c * ox - s * oy),
                north - (s * ox + c * oy),
                up - oz)

    # входы
    def _on_command(self, msg: DriverControllerCommand) -> None:
        t = _stamp_to_sec(msg.header.stamp)
        if not math.isfinite(t):
            self._fault = "некорректная метка времени"
            return
        self._advance(t)
        notch = float(msg.position)
        if not math.isfinite(notch):
            self._fault = "некорректный нотч"
        else:
            clamped = min(max(notch, self._command_min), self._command_max)
            if clamped != notch:
                self._fault = "нотч вне диапазона"
            self._notch = clamped
        self._publish(msg.header.stamp, t)

    def _on_wheel(self, which: str, msg: VelocitySensor) -> None:
        t = _stamp_to_sec(msg.header.stamp)
        if not math.isfinite(t):
            self._fault = "некорректная метка времени"
            return
        self._advance(t)
        raw = float(msg.velocity)
        if not math.isfinite(raw):
            self._fault = "нечисловая скорость колёс"
        else:
            verdict = self._wheels_slip.feed(
                which, t, raw * self._wheel_scale, max(self._state.velocity, 0.0),
                self._model_accel, self._notch)
            self._wheel_trust = verdict.trust
            self._slip = verdict.slip
            self._slip_mps = verdict.innovation
            self._slip_ratio = verdict.ratio
            self._slip_kind = verdict.kind
            self._fault = verdict.fault
            if verdict.speed is not None:
                self._wheels[which] = (t, max(raw * self._wheel_scale, 0.0))
                self._adapt_adhesion(t, verdict.speed)
                self._trust_t = t
                self._correct(verdict.speed)
                sigma = self._wheel_sigma / max(verdict.trust, 0.05)
                self._ekf.update_speed(verdict.speed, sigma)
        self._publish(msg.header.stamp, t)

    def _adapt_adhesion(self, t: float, v_wheel: float) -> None:

        if abs(self._notch) < 1.0 or self._trust_t is None:
            return
        dt = min(max(t - self._trust_t, 0.0), 0.5)
        if dt == 0.0:
            return

        err = max(self._state.velocity, 0.0) - v_wheel
        step = dt / self._adapt_tau * max(-0.05, min(0.05, -0.1 * err))
        self._adh_scale = min(1.2, max(0.5, self._adh_scale + step))

    # GNSS
    def _on_gnss(self, which: str, msg: NavSatFix) -> None:
        if msg.status.status < 0 or not math.isfinite(msg.latitude):
            return
        t = _stamp_to_sec(msg.header.stamp)
        sigma = self._gnss_pos_sigma
        if msg.position_covariance_type > 0 and msg.position_covariance[0] > 0.0:
            sigma = max(math.sqrt(msg.position_covariance[0]), 0.2)
        sample = (t, msg.latitude, msg.longitude, msg.altitude, sigma)
        if self._enu is None:
            self._enu = LocalEnu(*sample[1:4])
            if self._route_map.routes:
                self._route_map.set_frame(self._enu.to_enu)
                self._publish_route_map()
        if which == "master":
            self._master = sample
        else:
            self._rover = sample

        pair = None
        if (self._master is not None and self._rover is not None
                and abs(self._master[0] - self._rover[0]) <= self._gnss_pair_max_dt):
            m_e, m_n, m_u = self._enu.to_enu(*self._master[1:4])
            r_e, r_n, r_u = self._enu.to_enu(*self._rover[1:4])
            baseline = math.hypot(r_e - m_e, r_n - m_n)
            if abs(baseline - self._antenna_separation) <= 0.25 * self._antenna_separation:
                baseline_enu = math.atan2(r_n - m_n, r_e - m_e)
                self._baseline_yaw = (t, wrap_angle(baseline_enu - self._baseline_in_body))
                pair = (m_e, m_n, m_u, r_e, r_n, r_u)

        if which != "master" and pair is None:
            return
        if self._master is None:
            return

        m_e, m_n, m_u = self._enu.to_enu(*self._master[1:4])
        yaw_gnss = self._gnss_heading(t)
        yaw_for_offset = yaw_gnss if yaw_gnss is not None else self._ekf.yaw
        base_e, base_n, _ = self._lever_to_base(m_e, m_n, m_u, yaw_for_offset, self._master_offset)
        if pair is not None and yaw_gnss is not None:
            r_e, r_n, r_u = pair[3], pair[4], pair[5]
            re, rn, _ = self._lever_to_base(r_e, r_n, r_u, yaw_for_offset, self._rover_offset)
            base_e = 0.5 * (base_e + re)
            base_n = 0.5 * (base_n + rn)

        if self._gnss_track is not None:
            de, dn = base_e - self._gnss_track[1], base_n - self._gnss_track[2]
            if math.hypot(de, dn) >= 1.0:
                self._gnss_track_yaw = (t, math.atan2(dn, de))
                self._gnss_track = (t, base_e, base_n)
        else:
            self._gnss_track = (t, base_e, base_n)

        if self._publish_reference:
            self._append_pose(self._ref_path, msg.header.stamp, base_e, base_n, yaw_for_offset)

        if self._use_gnss_init and not self._gnss_locked:
            if yaw_gnss is None:
                return
            self._ekf.reset_pose(base_e, base_n, yaw_gnss, sigma_pos=sigma,
                                 sigma_yaw=self._gnss_yaw_sigma)
            self._gnss_locked = True
            self._path.poses.clear()
            self._route_path.poses.clear()
            self._follower.init_pose(base_e, base_n, yaw_gnss)
            self._try_route_match(base_e, base_n, from_gnss=True)
            self.get_logger().info(
                f"GNSS-выставка: ({base_e:.1f}, {base_n:.1f}) м, курс "
                f"{math.degrees(yaw_gnss):.1f}° (ENU)")
            self._log_route_proximity(base_e, base_n)
            return

        self._try_route_match(base_e, base_n, from_gnss=True)

        if self._t_last is None:
            return
        if self._gnss_fusion_max_time > 0 and self._t_first is not None \
                and t - self._t_first > self._gnss_fusion_max_time:
            return
        if abs(t - self._t_last) > self._gnss_max_age:
            return
        if self._gnss_fusion:
            self._fuse_gnss(t, base_e, base_n, sigma, yaw_gnss)

    def _heading_candidates(self) -> list[float]:

        yaws = []
        if self._heading_from_baseline and self._baseline_yaw is not None:
            yaws.append(self._baseline_yaw[1])
        if self._gnss_track_yaw is not None:
            yaws.append(self._gnss_track_yaw[1])
        yaws.append(self._ekf.yaw)
        return yaws

    def _try_route_match(self, x: float, y: float, from_gnss: bool) -> None:
        if self._follower.locked:
            return
        speed = max(self._state.velocity, 0.0)
        yaws = self._heading_candidates()
        if self._follower.try_match(x, y, yaws, speed, require_motion=not from_gnss):
            r = self._follower.route
            self._follower_route_name = r.name
            self._copy_lock(self._result_follower, self._follower)
            self._result_route_name = r.name
            self._gnss_along = None
            self._result_speed = self._ekf.speed
            self._route_path.poses.clear()
            self._path.poses.clear()
            self.get_logger().info(
                f"Привязка к маршруту «{r.name}» по {'GNSS' if from_gnss else 'счислению'}: "
                f"s={self._follower.s:.0f} м из {r.length:.0f} м")

    def _log_route_proximity(self, x: float, y: float) -> None:
        if not self._route_map.ready:
            self.get_logger().warn("Карта маршрутов ещё не переведена в ENU")
            return
        near = self._route_map.nearest(x, y)
        if near is None:
            self.get_logger().warn("В карте нет точек маршрута")
            return
        r, s, lateral, route_yaw = near
        dyaw = min(abs(wrap_angle(route_yaw - yaw)) for yaw in self._heading_candidates())
        self.get_logger().info(
            f"Ближайший маршрут «{r.name}»: {lateral:.1f} м, s={s:.0f}/{r.length:.0f} м, "
            f"Δкурс={math.degrees(dyaw):.0f}° "
            f"(порог привязки {self._follower.p.max_lateral_m:.0f} м / "
            f"{math.degrees(self._follower.p.max_heading_rad):.0f}°)")

    def _publish_route_map(self) -> None:
        markers = MarkerArray()
        palette = ((0.6, 0.6, 0.6), (0.35, 0.35, 0.45))
        for i, r in enumerate(self._route_map.routes):
            m = Marker()
            m.header.frame_id = self._map_frame
            m.ns = "route_map"
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 1.0
            c = palette[i % len(palette)]
            m.color.r, m.color.g, m.color.b, m.color.a = c[0], c[1], c[2], 0.9
            m.pose.orientation.w = 1.0
            m.points = [Point(x=float(px), y=float(py), z=0.0)
                        for px, py in r.json_xy[::2]]
            markers.markers.append(m)
        self._pub_route_map.publish(markers)

    def _gnss_heading(self, t: float):

        if self._baseline_yaw is not None and t - self._baseline_yaw[0] <= self._gnss_max_age:
            self._heading_from_baseline = True
            return self._baseline_yaw[1]
        self._heading_from_baseline = False
        if (self._gnss_track_yaw is not None and t - self._gnss_track_yaw[0] <= 2.0
                and self._ekf.speed >= self._pp_min_speed):
            return self._gnss_track_yaw[1]
        return None

    def _copy_lock(self, dst: RouteFollower, src: RouteFollower) -> None:
        dst.route = src.route
        dst.s = src.s
        dst.x, dst.y, dst.yaw, dst.curv = src.x, src.y, src.yaw, src.curv
        dst.initialized = True

    def _fuse_gnss_speed(self, t: float, base_e: float, base_n: float) -> None:

        route = self._result_follower.route
        if route is None:
            return
        s, lateral, _, _, _ = route.project(base_e, base_n)
        if lateral > self._follower.p.max_lateral_m:
            self._gnss_along = None
            return
        prev = self._gnss_along
        self._gnss_along = (t, s)
        if prev is None:
            return
        dt = t - prev[0]
        ds = s - prev[1]
        if 0.02 < dt < 1.0 and abs(ds) <= 15.0:
            v_red = max(ds / dt, 0.0)
            if abs(v_red - self._ekf.speed) < 3.0:
                self._ekf.update_speed(v_red, self._gnss_speed_sigma)
        self._gnss_rejects = 0
        self._gnss_fused_count += 1
        self._gnss_last_fused_t = t

    def _fuse_gnss(self, t: float, base_e: float, base_n: float, sigma: float, yaw_gnss) -> None:

        if self._result_follower.locked:
            self._fuse_gnss_speed(t, base_e, base_n)
            return
        accepted = self._ekf.update_position(base_e, base_n, sigma, gated=False)
        if not accepted:
            self._gnss_rejects += 1
            if self._gnss_rejects >= self._gnss_reject_reset:
                self.get_logger().warn(
                    f"GNSS отброшен {self._gnss_rejects} раз подряд: фильтр расходится, "
                    "повторная выставка по GNSS")
                self._ekf.reset_pose(base_e, base_n,
                                     yaw_gnss if yaw_gnss is not None else self._ekf.yaw,
                                     sigma_pos=sigma, sigma_yaw=0.2)
                self._gnss_rejects = 0
            return
        self._gnss_rejects = 0
        self._gnss_fused_count += 1
        self._gnss_last_fused_t = t

        if yaw_gnss is not None:
            if self._heading_from_baseline:
                self._ekf.update_heading(yaw_gnss, self._gnss_yaw_sigma)
            if not self._follower.locked:
                v = self._ekf.speed
                if v >= self._pp_min_speed:
                    lookahead = max(self._pp_lookahead_min, self._pp_lookahead_time * v)
                    tx = base_e + lookahead * math.cos(yaw_gnss)
                    ty = base_n + lookahead * math.sin(yaw_gnss)
                    self._ekf.update_pure_pursuit(tx, ty, self._pp_kappa_sigma)

    # модель
    def _advance(self, t: float) -> None:

        if not math.isfinite(t):
            return
        if self._t_last is None:
            self._t_first = self._t_last = t
            return
        dt = t - self._t_last
        if dt <= 0.0:
            return
        if dt > self._max_gap:
            self.get_logger().warn(f"Разрыв во входных данных {dt:.2f} с: интегрирование пропущено")
            self._t_last = t
            return

        if self._input_period is None:
            self._input_period = dt
        else:
            self._input_period += 0.2 * (dt - self._input_period)
        u = model.Inputs(notch=self._notch, tram_weight=self._mass,
                         track_slope=self._slope, adh_cond=self._adh_cond,
                         adh_scale=self._adh_scale,
                         traction_torque_per_notch_nm=self._traction_torque,
                         brake_torque_per_notch_nm=self._brake_torque,
                         max_brake_torque_nm=self._brake_torque_limit)
        n = max(1, math.ceil(dt / self._rk3_h_max))
        h = dt / n
        state = self._state
        y = None
        for _ in range(n):
            state, y = model.step(state, u, h)

        velocity = max(state.velocity, 0.0)
        omega = max(state.omega, 0.0)
        torque = state.motor_torque
        if not all(map(math.isfinite, (velocity, omega, torque, state.distance))):
            self.get_logger().error(
                "Состояние модели не конечно, сброс к последнему измерению колёс")
            v_wheel = max((v for tv in self._wheels.values() if tv for _, v in [tv]), default=0.0)
            velocity, omega, torque = v_wheel, v_wheel / model.R, 0.0
            state = replace(state, distance=self._state.distance)
            y = None
        elif abs(model.R * omega - velocity) > self._slip_resync:

            omega = velocity / model.R
        self._state = replace(state, velocity=velocity, omega=omega, motor_torque=torque)
        self._t_last = t

        self._ekf.predict(dt)
        if y is not None and math.isfinite(y.get("adhesion_force_n", math.nan)):
            weight = max(self._mass * model.G, 1.0)
            self._mu = y["adhesion_force_n"] / weight
            self._model_slip = y["slip_m_s"]
        if y is not None and math.isfinite(y["acceleration_m_s2"]):

            self._model_accel = y["acceleration_m_s2"]
            self._ekf.update_accel(self._model_accel, self._accel_sigma)

        if not self._follower.locked and self._position_source == "ekf":
            ex, ey = self._ekf.position
            self._follower.set_prelock_pose(ex, ey, self._ekf.yaw)
            self._try_route_match(ex, ey, from_gnss=False)
        elif self._follower.locked:
            v_model = max(self._state.velocity, 0.0)
            scale = self._route_distance_scale
            self._follower.advance(v_model * dt * scale, v_model)
            if self._follower.route.name != self._follower_route_name:
                self._follower_route_name = self._follower.route.name
                self.get_logger().info(
                    f"Конечная: синяя на маршрут «{self._follower_route_name}»")
                self._route_path.poses.clear()
            g = self._result_follower
            if g.locked:
                v_along = self._ekf.speed
                ds = v_along * dt * scale

                if (self._gnss_along is not None
                        and t - self._gnss_along[0] <= self._gnss_max_age
                        and g.route.name == self._result_route_name):
                    err = self._gnss_along[1] - (g.s + ds)
                    if abs(err) < 40.0:
                        ds += err * (dt / 1.5)
                        ds = max(ds, 0.0)
                        v_along = ds / dt
                self._result_speed = v_along
                g.advance(ds, v_along)
                if g.route.name != self._result_route_name:
                    self._result_route_name = g.route.name
                    self._gnss_along = None
                    self.get_logger().info(
                        f"Конечная: зелёная на маршрут «{self._result_route_name}»")
                    self._path.poses.clear()
                self._ekf.snap_pose(g.x, g.y, g.yaw, g.curv)

    # публикация
    def _publish_diagnostics(self, stamp, t: float) -> None:

        status = DiagnosticStatus()
        status.name = "tram_model slip"
        status.hardware_id = "bogies"
        fresh = any(tv is not None and t - tv[0] <= self._wheel_max_age
                    for tv in self._wheels.values())
        outlier = self._fault.startswith((
            "выброс", "скачок", "нечисло", "некоррект", "одометрия"))
        if outlier:
            status.level = DiagnosticStatus.ERROR
        elif not fresh:
            status.level = DiagnosticStatus.STALE
        elif self._slip or self._wheel_trust < 0.8 or self._fault:
            status.level = DiagnosticStatus.WARN
        else:
            status.level = DiagnosticStatus.OK
        status.message = self._fault or ("проскальзывание" if self._slip else "норма")
        status.values = [
            KeyValue(key="slip", value="1" if self._slip else "0"),
            KeyValue(key="slip_kind", value=self._slip_kind),
            KeyValue(key="slip_ratio", value=f"{self._slip_ratio:.4f}"),
            KeyValue(key="slip_obs_mps", value=f"{self._slip_mps:.4f}"),
            KeyValue(key="slip_model_mps", value=f"{self._model_slip:.4f}"),
            KeyValue(key="adhesion", value=f"{self._mu:.4f}"),
            KeyValue(key="adhesion_scale", value=f"{self._adh_scale:.4f}"),
            KeyValue(key="wheel_trust", value=f"{self._wheel_trust:.4f}"),
            KeyValue(key="bogie_front", value=self._wheels_slip.front.kind),
            KeyValue(key="bogie_rear", value=self._wheels_slip.rear.kind),
        ]
        msg = DiagnosticArray()
        msg.header.stamp = stamp
        msg.header.frame_id = self._base_frame
        msg.status.append(status)
        self._pub_diag.publish(msg)
        flag = Bool()
        flag.data = self._slip
        self._pub_slip.publish(flag)

    def _correct(self, v_wheel: float) -> None:

        g = self._gain * self._wheel_trust
        s = self._state
        self._state = replace(
            s,
            velocity=max(s.velocity + g * (v_wheel - s.velocity), 0.0),
            omega=max(s.omega + g * (v_wheel / model.R - s.omega), 0.0),
        )

    def _to_route_map(self, x: float, y: float, yaw: float):

        lat, lon, _ = self._enu.to_geodetic(x, y)
        rx, ry = geodetic_to_json_xy(lat, lon, **self._route_json_params)
        return rx, ry, wrap_angle(yaw + self._route_json_params["angle"])

    def _append_pose(self, path: Path, stamp, x: float, y: float, yaw: float) -> None:
        if self._enu is None:
            return
        x, y, yaw = self._to_route_map(x, y, yaw)
        if path.poses:
            last = path.poses[-1].pose.position
            if math.hypot(x - last.x, y - last.y) < self._path_min_step:
                return
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(pose)
        if len(path.poses) > self._path_max_points:
            del path.poses[0]
        path.header.stamp = stamp

    def _publish_paths(self) -> None:

        if self._path.poses:
            self._pub_path.publish(self._path)
        if self._publish_reference and self._ref_path.poses:
            self._pub_ref_path.publish(self._ref_path)
        if self._route_path.poses:
            self._pub_route_path.publish(self._route_path)

    def _make_odom(self, stamp, x: float, y: float, yaw: float, v: float,
                   yaw_rate: float) -> Odometry:
        x, y, yaw = self._to_route_map(x, y, yaw)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = yaw_rate
        return odom

    def _publish(self, stamp, t: float) -> None:
        ekf = self._ekf
        v_model = max(self._state.velocity, 0.0)

        v = ekf.speed if self._velocity_source == "ekf" else v_model

        vel = VelocitySensor()
        vel.header.stamp = stamp
        vel.header.frame_id = self._base_frame
        vel.velocity = v
        self._pub_velocity.publish(vel)

        if self._enu is None:
            return

        f = self._follower
        g = self._result_follower
        if self._position_source == "route":
            if not f.locked:
                return
            ex, ey, yaw, kappa = f.x, f.y, f.yaw, f.curv
            odom_speed = v_model
        elif g.locked:
            ex, ey, yaw, kappa = g.x, g.y, g.yaw, g.curv
            odom_speed = v
        else:
            ex, ey = ekf.position
            yaw, kappa = ekf.yaw, ekf.kappa
            odom_speed = v
        odom = self._make_odom(stamp, ex, ey, yaw, odom_speed, odom_speed * kappa)
        if self._position_source == "ekf":
            P = ekf.P
            cov = odom.pose.covariance
            angle = self._route_json_params["angle"]
            scale = self._route_json_params["scale"]
            c, s = math.cos(angle), math.sin(angle)
            transform = scale * np.array(((c, -s), (s, c)))
            xy_cov = transform @ P[:2, :2] @ transform.T
            cov[0], cov[1], cov[6], cov[7] = (
                xy_cov[0, 0], xy_cov[0, 1], xy_cov[1, 0], xy_cov[1, 1])
            cov[35] = P[2, 2]
            odom.twist.covariance[0] = P[4, 4]
        self._append_pose(self._path, stamp, ex, ey, yaw)
        self._pub_position.publish(odom)
        if f.locked:
            route_odom = self._make_odom(
                stamp, f.x, f.y, f.yaw, v_model, v_model * f.curv)
            self._pub_route_position.publish(route_odom)
            self._append_pose(self._route_path, stamp, f.x, f.y, f.yaw)
        qz = odom.pose.pose.orientation.z
        qw = odom.pose.pose.orientation.w

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._map_frame
        tf.child_frame_id = self._base_frame
        tf.transform.translation.x = odom.pose.pose.position.x
        tf.transform.translation.y = odom.pose.pose.position.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf.sendTransform(tf)

        gnss = "нет"
        if self._gnss_last_fused_t is not None:
            age = t - self._gnss_last_fused_t
            gnss = "есть" if age <= self._gnss_max_age else f"потерян {age:.0f} с"
        if g.locked:
            route = f"{g.route.name} s={g.s:.0f} м (синяя {f.s:.0f})"
        elif f.locked:
            route = f"{f.route.name} s={f.s:.0f} м"
        else:
            near = self._route_map.nearest(ex, ey) if self._route_map.ready else None
            if near is None:
                route = "нет привязки"
            else:
                r, s, lateral, _ = near
                route = f"нет привязки, ближайший {r.name} {lateral:.0f} м (s={s:.0f})"
        self._publish_diagnostics(stamp, t)
        hz = 1.0 / self._input_period if self._input_period else 0.0
        self.get_logger().info(
            f"t={t:.2f} с | вход={hz:4.1f} Гц | нотч={self._notch:+.0f} | "
            f"v={v:5.2f} м/с | a={ekf.accel:+5.2f} | "
            f"EKF (ENU): x={ex:8.1f} y={ey:8.1f} "
            f"курс={math.degrees(wrap_angle(ekf.yaw)):6.1f}° "
            f"k={ekf.kappa:+.4f} | слип={self._slip_mps:+.2f} "
            f"доверие={self._wheel_trust:.2f} | GNSS: {gnss} | карта: {route} "
            f"({f.x:8.1f}, {f.y:8.1f})",
            throttle_duration_sec=self._log_period)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TramModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
