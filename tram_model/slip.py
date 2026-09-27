from dataclasses import dataclass

TRUSTED, FAULT, RECOVERING = 0, 1, 2
_KIND_FAULT = {
    "slip": "проскальзывание",
    "slide": "юз",
    "stuck": "залипание датчика",
    "outlier": "скачок скорости колёс",
}


# параметры
@dataclass(frozen=True)
class SlipConfig:
    disagreement: float
    innovation_limit: float
    speed_cap: float
    accel_traction: float = 1.9
    accel_brake: float = 2.8
    accel_emergency: float = 6.0
    accel_jump: float = 6.0
    diff_abs: float = 0.2
    diff_rel: float = 0.015
    onset: float = 0.07
    accel_margin: float = 1.2
    stuck_samples: int = 6
    hold_s: float = 0.4
    recover_s: float = 1.5
    other_age_s: float = 0.6


@dataclass(frozen=True)
class SampleVerdict:
    speed: float | None
    trust: float
    slip: bool
    kind: str
    ratio: float
    fault: str
    innovation: float


def _slope(pts):
    n = len(pts)
    if n < 2:
        return None
    mt = sum(p[0] for p in pts) / n
    mz = sum(p[1] for p in pts) / n
    stt = sum((p[0] - mt) ** 2 for p in pts)
    if stt < 1e-4:
        return None
    return sum((p[0] - mt) * (p[1] - mz) for p in pts) / stt


class _Bogie:
    def __init__(self):
        self.state = TRUSTED
        self.trust = 1.0
        self.kind = "ok"
        self.hist = []
        self.same_count = 0
        self.same_any = 0
        self.t_clear = None
        self.t_recover = None

    def accel(self, t, z, span):
        pts = [(tt, zz) for tt, zz in self.hist if t - tt <= span + 1e-6]
        pts.append((t, z))
        return _slope(pts)


# детектор
class SlipDetector:
    def __init__(self, cfg: SlipConfig):
        self.cfg = cfg
        self.front = _Bogie()
        self.rear = _Bogie()

    def feed(self, which: str, t: float, z: float, v_model: float,
             a_model: float, notch: float) -> SampleVerdict:
        cfg = self.cfg
        bogie = self.front if which == "front" else self.rear
        other = self.rear if which == "front" else self.front
        innovation = z - v_model
        if z < -1.0 or z > cfg.speed_cap:
            return self._verdict(None, bogie, other, True, "outlier", 0.0,
                                 innovation, "выброс скорости колёс")
        z = max(z, 0.0)
        innovation = z - v_model

        prev = bogie.hist[-1] if bogie.hist else None
        if prev is not None and z == prev[1] and abs(z) > 0.1:
            bogie.same_count += 1
        else:
            bogie.same_count = 0
        if prev is not None and z == prev[1]:
            bogie.same_any += 1
        else:
            bogie.same_any = 0

        a_w = bogie.accel(t, z, 0.3)
        a_long = bogie.accel(t, z, 0.8)
        a_jump = None
        if prev is not None and t - prev[0] > 1e-3:
            a_jump = (z - prev[1]) / (t - prev[0])
        bogie.hist.append((t, z))
        if len(bogie.hist) > 8:
            del bogie.hist[0]

        other_z = other_e = other_acc = None
        other_fresh = False
        if other.hist:
            ot, oz = other.hist[-1]
            age = t - ot
            if 0.0 <= age <= cfg.other_age_s:
                other_z = oz
                other_e = oz - v_model
                other_pts = [(tt, zz) for tt, zz in other.hist if t - tt <= 0.8 + 1e-6]
                other_acc = _slope(other_pts)
                other_fresh = other.state != FAULT

        va = abs(v_model)
        diff_thr = cfg.diff_abs + cfg.diff_rel * va
        disagree = other_z is not None and abs(z - other_z) > diff_thr
        a_lo = -cfg.accel_brake
        jump_max = cfg.accel_jump
        if (other_fresh and other_z is not None and abs(z - other_z) <= diff_thr
                and a_w is not None and other_acc is not None
                and a_w < 0.0 and other_acc < 0.0 and abs(a_w - other_acc) < 2.0):
            a_lo = -cfg.accel_emergency
            jump_max = cfg.accel_jump + 2.0
        phys_bad = (
            (a_w is not None and (a_w > cfg.accel_traction or a_w < a_lo))
            or (a_jump is not None and abs(a_jump) > jump_max)
        )
        stuck = bogie.same_count >= cfg.stuck_samples and (
            abs(a_model) > 0.15
            or (other_z is not None and abs(other_z - z) > cfg.diff_abs))
        frozen0 = (
            bogie.same_any >= cfg.stuck_samples and abs(z) <= 0.1 and notch > 0
            and other_z is not None and other_z > cfg.diff_abs + 0.1
            and other_acc is not None and a_model > 0.2
            and abs(other_acc - a_model) < 0.6
            and 0.05 < other_acc < cfg.accel_traction)
        stuck = stuck or frozen0

        blamed = False
        if disagree and other_e is not None:
            if abs(innovation) > 1.3 * abs(other_e) + 0.05:
                blamed = True
            elif abs(other_e) <= 1.3 * abs(innovation) + 0.05:
                blamed = ((notch > 0 and z > other_z) or (notch < 0 and z < other_z))
            if other_acc is not None and a_long is not None and abs(a_model) > 0.15:
                my_dev = abs(a_long - a_model)
                ot_dev = abs(other_acc - a_model)
                if my_dev < 0.45 and ot_dev > my_dev + 0.5:
                    blamed = False
                elif ot_dev < 0.45 and my_dev > ot_dev + 0.5:
                    blamed = True

        if other_fresh:
            anomaly = stuck or phys_bad or (blamed and abs(innovation) > cfg.onset)
        else:
            r_acc = (abs(a_w - a_model) / cfg.accel_margin) if a_w is not None else 0.0
            r_innov = abs(innovation) / max(cfg.innovation_limit, 1e-6)
            anomaly = stuck or phys_bad or (r_innov > 1.5 and r_acc > 1.5)

        self._step_state(bogie, t, anomaly, notch, innovation, stuck, a_w, a_lo,
                         cfg.accel_traction)
        ratio = innovation / max(va, 1.0)
        if bogie.state == FAULT:
            return self._verdict(None, bogie, other, True, bogie.kind, ratio,
                                 innovation, _KIND_FAULT.get(bogie.kind, bogie.kind))

        speed = z
        if (other.hist and other.state != FAULT and other_z is not None
                and not disagree):
            speed = 0.5 * (z + other_z)
        return self._verdict(speed, bogie, other, self._any_fault(), bogie.kind,
                             ratio, innovation, "")

    def _step_state(self, bogie, t, anomaly, notch, innovation, stuck, a_w, a_lo, a_hi):
        cfg = self.cfg
        if anomaly:
            if bogie.state != FAULT:
                bogie.state = FAULT
                bogie.t_clear = None
                if stuck:
                    bogie.kind = "stuck"
                elif notch > 0 and innovation > 0:
                    bogie.kind = "slip"
                elif notch < 0 and innovation < 0:
                    bogie.kind = "slide"
                else:
                    bogie.kind = "outlier"
        elif bogie.state == FAULT:
            plausible = a_w is None or (0.8 * a_lo < a_w < 0.8 * a_hi)
            consistent = abs(innovation) < 0.6 * cfg.innovation_limit
            if plausible and consistent:
                if bogie.t_clear is None:
                    bogie.t_clear = t
                if t - bogie.t_clear >= cfg.hold_s:
                    bogie.state = RECOVERING
                    bogie.t_recover = t
            else:
                bogie.t_clear = None
        elif bogie.state == RECOVERING:
            if t - bogie.t_recover >= cfg.recover_s:
                bogie.state = TRUSTED
                bogie.kind = "ok"
        if bogie.state == FAULT:
            bogie.trust = 0.0
        elif bogie.state == RECOVERING:
            bogie.trust = min(1.0, max(0.05, (t - bogie.t_recover) / cfg.recover_s))
        else:
            bogie.trust = 1.0
            if not anomaly:
                bogie.kind = "ok"

    def _any_fault(self) -> bool:
        return self.front.state == FAULT or self.rear.state == FAULT

    def _verdict(self, speed, bogie, other, slip, kind, ratio, innovation, fault):
        trust = bogie.trust if speed is not None else 0.0
        if speed is not None and other.state == RECOVERING:
            trust = min(trust, other.trust)
        return SampleVerdict(speed, trust, slip, kind, ratio, fault, innovation)
