import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path


def _find_config_path() -> Path:
    here = Path(__file__).resolve()
    for base in here.parents:
        installed = base / "share" / "tram_model" / "json" / "tram.json"
        if installed.is_file():
            return installed
    dev = here.parent.parent / "json" / "tram.json"
    if dev.is_file():
        return dev
    raise FileNotFoundError(
        "Не найден tram.json; ожидается json/tram.json рядом с пакетом "
        "или share/tram_model/json/tram.json после установки"
    )


def _load_tram_config() -> dict:
    with _find_config_path().open(encoding="utf-8") as stream:
        return json.load(stream)


# параметры
CONFIG = _load_tram_config()
_DATASET = CONFIG["dataset"]
_VEHICLE = CONFIG["vehicle_reference"]
_DRIVE = CONFIG["drive_reference"]
_CALIBRATION = CONFIG.get("dynamics_calibration", {})
_BRAKES = CONFIG["brakes_reference"]
_OBSERVER = CONFIG["observer"]

DT = 0.001
SIM_TIME = 50.0
PRINT_EVERY = 100
R = _VEHICLE["wheel_radius_new_m"]

K_POS = float(_CALIBRATION.get("traction_torque_per_notch_nm", 1449.0))
K_NEG = float(_CALIBRATION.get("brake_torque_per_notch_nm", 1176.0))
MAX_BRAKE_TORQUE = float(_CALIBRATION.get(
    "max_brake_torque_nm", K_NEG * abs(CONFIG["adapter"]["command_min"])))
MAX_POWER = _DRIVE["total_motor_nominal_power_w"]
NUM_WHEELS = _VEHICLE["wheel_count"]
NUM_TRACTION_WHEELS = _DATASET["driven_axle_count"] * 2
WHEEL_MASS = 195.0
G = _VEHICLE["standard_gravity_mps2"]
J = NUM_TRACTION_WHEELS * 0.5 * WHEEL_MASS * R**2

MOTOR_TAU_INV = 1.0 / _OBSERVER["drive_tau_s"]

_I_BRAKE = _BRAKES["braking_resistor_current_20s_max_a"]
_R_BRAKE = _BRAKES["braking_resistor_resistance_ohm"]
MAX_BRAKE_POWER = _I_BRAKE * _I_BRAKE * _R_BRAKE

ADH_COND = 0
NOTCH = -7.0
TRAM_WEIGHT = _DATASET["empty_mass_kg"]
TRACK_SLOPE = 0.0
VEL_INIT = 10.0
POS_INIT = 0.0


# входы и состояние
@dataclass(frozen=True)
class Inputs:
    notch: float = NOTCH
    tram_weight: float = TRAM_WEIGHT
    track_slope: float = TRACK_SLOPE
    adh_cond: int = ADH_COND
    adh_scale: float = 1.0
    traction_torque_per_notch_nm: float = K_POS
    brake_torque_per_notch_nm: float = K_NEG
    max_brake_torque_nm: float = MAX_BRAKE_TORQUE


@dataclass(frozen=True)
class State:

    velocity: float = VEL_INIT
    distance: float = POS_INIT
    omega: float = VEL_INIT / R
    motor_torque: float = 0.0


# расчёт шага
def outputs(state: State, u: Inputs) -> dict:

    if u.tram_weight <= 0:
        raise ValueError("tram_weight должна быть положительной массой в кг")

    v = max(state.velocity, 0.0)
    omega = max(state.omega, 0.0)
    slip = R * omega - v

    if u.adh_cond == 0:
        a, b, c, d = 0.54, 1.2, 1.0, 1.0
    elif u.adh_cond == 1:
        a, b, c, d = 0.54, 1.2, 0.2, 0.2
    elif u.adh_cond == 2:
        a, b, c, d = 0.54, 1.2, 0.1, 0.1
    else:
        a, b, c, d = 0.05, 0.5, 0.08, 0.08

    normal_per_wheel = u.tram_weight * G / NUM_WHEELS
    adhesion_per_wheel = normal_per_wheel * (
        c * math.exp(-a * slip) - d * math.exp(-b * slip)
    )
    adhesion_total = NUM_TRACTION_WHEELS * adhesion_per_wheel * u.adh_scale

    torque_per_notch = (
        u.traction_torque_per_notch_nm if u.notch > 0
        else u.brake_torque_per_notch_nm)
    requested_torque = torque_per_notch * u.notch
    requested_torque = max(requested_torque, -u.max_brake_torque_nm)
    if omega > 1e-3:
        power = requested_torque * omega
        if u.notch > 0 and power >= MAX_POWER:
            requested_torque = MAX_POWER / omega
        elif u.notch <= 0 and power <= -MAX_BRAKE_POWER:
            requested_torque = -MAX_BRAKE_POWER / omega

    torque_rate = MOTOR_TAU_INV * (requested_torque - state.motor_torque)
    wheel_acceleration = (state.motor_torque - R * adhesion_total) / J

    if v > 0:
        resistance = (0.0147 * u.tram_weight + 125.83 * v
                      + u.tram_weight * G * math.sin(u.track_slope))
    else:
        resistance = 0.0
    acceleration = (adhesion_total - resistance) / u.tram_weight

    return {
        "distance_m": state.distance,
        "speed_m_s": v,
        "acceleration_m_s2": acceleration,
        "wheel_speed_rad_s": omega,
        "slip_m_s": slip,
        "adhesion_force_n": adhesion_total,
        "resistance_n": resistance,
        "motor_torque_nm": state.motor_torque,
        "_torque_rate": torque_rate,
        "_wheel_acceleration": wheel_acceleration,
    }


def _derivative(state: State, u: Inputs) -> State:
    y = outputs(state, u)
    return State(y["acceleration_m_s2"], y["speed_m_s"],
                 y["_wheel_acceleration"], y["_torque_rate"])


def _add(state: State, derivative: State, scale: float) -> State:
    return State(state.velocity + scale * derivative.velocity,
                 state.distance + scale * derivative.distance,
                 state.omega + scale * derivative.omega,
                 state.motor_torque + scale * derivative.motor_torque)


def step(state: State, u: Inputs, dt: float = DT) -> tuple[State, dict]:

    if dt <= 0:
        raise ValueError("dt должен быть положительным")
    k1 = _derivative(state, u)
    k2 = _derivative(_add(state, k1, dt / 2), u)
    k3 = _derivative(_add(state, k2, 3 * dt / 4), u)
    next_state = State(
        state.velocity + dt * (2*k1.velocity/9 + k2.velocity/3 + 4*k3.velocity/9),
        state.distance + dt * (2*k1.distance/9 + k2.distance/3 + 4*k3.distance/9),
        state.omega + dt * (2*k1.omega/9 + k2.omega/3 + 4*k3.omega/9),
        state.motor_torque + dt * (2*k1.motor_torque/9 + k2.motor_torque/3
                                   + 4*k3.motor_torque/9),
    )
    y = outputs(next_state, u)
    y.pop("_torque_rate")
    y.pop("_wheel_acceleration")
    return next_state, y


def main() -> None:
    state = State()
    if "--stream" in sys.argv:

        current = {"notch": NOTCH, "tram_weight": TRAM_WEIGHT,
                   "track_slope": TRACK_SLOPE, "adh_cond": ADH_COND,
                   "adh_scale": 1.0,
                   "traction_torque_per_notch_nm": K_POS,
                   "brake_torque_per_notch_nm": K_NEG,
                   "max_brake_torque_nm": MAX_BRAKE_TORQUE}
        for index, line in enumerate(sys.stdin, 1):
            if not line.strip():
                continue
            changed = json.loads(line)
            if not isinstance(changed, dict) or set(changed) - current.keys():
                raise ValueError("Ожидается JSON-объект с полями: " + ", ".join(current))
            current.update(changed)
            state, y = step(state, Inputs(**current))
            print(json.dumps({"time_s": index * DT, **y}), flush=True)
    else:
        for index in range(1, round(SIM_TIME / DT) + 1):
            u = Inputs(notch=NOTCH, tram_weight=TRAM_WEIGHT,
                       track_slope=TRACK_SLOPE, adh_cond=ADH_COND)
            state, y = step(state, u)
            if index % PRINT_EVERY == 0:
                print(f"t={index*DT:7.3f} с | x={y['distance_m']:9.3f} м | "

                      )


if __name__ == "__main__":
    main()
