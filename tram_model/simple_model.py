import math


class WheelIntegrator:

    def __init__(self, max_age_s=0.25, max_gap_s=3.0):
        if not all(math.isfinite(v) and v > 0 for v in (max_age_s, max_gap_s)):
            raise ValueError("Интервалы должны быть конечными и положительными")
        self.max_age_s = max_age_s
        self.max_gap_s = max_gap_s
        self.time = None
        self.distance = 0.0
        self.wheels = {"front": None, "rear": None}

    def speed_at(self, t):
        fresh = [v for sample in self.wheels.values() if sample is not None
                 for tw, v in [sample] if tw <= t < tw + self.max_age_s]
        return sum(fresh) / len(fresh) if fresh else 0.0

    @property
    def speed(self):
        return self.speed_at(self.time) if self.time is not None else 0.0

    def advance(self, t):

        if not math.isfinite(t):
            raise ValueError("Время должно быть конечным")
        if self.time is None:
            self.time = t
            return 0.0
        if t <= self.time:
            return 0.0
        start = self.time
        self.time = t
        if t - start > self.max_gap_s:
            self.wheels = dict.fromkeys(self.wheels)
            return 0.0

        cuts = sorted({start, t} | {
            sample[0] + self.max_age_s for sample in self.wheels.values()
            if sample is not None and start < sample[0] + self.max_age_s < t})
        ds = sum(self.speed_at((a + b) / 2) * (b - a)
                 for a, b in zip(cuts, cuts[1:]))
        self.distance += ds
        return ds

    def update(self, which, t, velocity):

        if which not in self.wheels:
            raise ValueError("Датчик должен быть front или rear")
        if not math.isfinite(t) or not math.isfinite(velocity):
            return 0.0
        if self.time is not None and t < self.time:
            return 0.0
        ds = self.advance(t)
        self.wheels[which] = (t, max(velocity, 0.0))
        return ds
