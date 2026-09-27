import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)
_UTM_K0 = 0.9996

JSON_ORIGIN_LAT = 55.8104031450
JSON_ORIGIN_LON = 37.4623050517
JSON_X0 = 103634.578
JSON_Y0 = 86048.242
JSON_ANGLE = -0.021921875
JSON_SCALE = 0.999461


# координаты
def _geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sl * sl)
    x = (n + alt_m) * cl * math.cos(lon)
    y = (n + alt_m) * cl * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + alt_m) * sl
    return x, y, z


def _ecef_to_geodetic(x: float, y: float, z: float):

    a = _WGS84_A
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - _WGS84_E2))
    for _ in range(6):
        sl, cl = math.sin(lat), math.cos(lat)
        n = a / math.sqrt(1.0 - _WGS84_E2 * sl * sl)
        lat = math.atan2(z + _WGS84_E2 * n * sl, p)
    sl, cl = math.sin(lat), math.cos(lat)
    n = a / math.sqrt(1.0 - _WGS84_E2 * sl * sl)
    alt = p / cl - n if abs(cl) > 1e-12 else abs(z) / sl - n * (1.0 - _WGS84_E2)
    return math.degrees(lat), math.degrees(math.atan2(y, x)), alt


def enu_to_geodetic(east: float, north: float, up: float,
                    lat0: float, lon0: float, alt0: float = 0.0):
    x0, y0, z0 = _geodetic_to_ecef(lat0, lon0, alt0)
    lat, lon = math.radians(lat0), math.radians(lon0)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    dx = -so * east - sl * co * north + cl * co * up
    dy = co * east - sl * so * north + cl * so * up
    dz = cl * north + sl * up
    return _ecef_to_geodetic(x0 + dx, y0 + dy, z0 + dz)


def json_xy_to_geodetic(x: float, y: float,
                        origin_lat: float = JSON_ORIGIN_LAT,
                        origin_lon: float = JSON_ORIGIN_LON,
                        x0: float = JSON_X0, y0: float = JSON_Y0,
                        angle: float = JSON_ANGLE, scale: float = JSON_SCALE):

    dx = (x - x0) / scale
    dy = (y - y0) / scale
    c, s = math.cos(angle), math.sin(angle)
    east = c * dx + s * dy
    north = -s * dx + c * dy
    lat, lon, _ = enu_to_geodetic(east, north, 0.0, origin_lat, origin_lon, 0.0)
    return lat, lon


def geodetic_to_json_xy(lat_deg: float, lon_deg: float,
                        origin_lat: float = JSON_ORIGIN_LAT,
                        origin_lon: float = JSON_ORIGIN_LON,
                        x0: float = JSON_X0, y0: float = JSON_Y0,
                        angle: float = JSON_ANGLE, scale: float = JSON_SCALE):

    x, y, z = _geodetic_to_ecef(lat_deg, lon_deg, 0.0)
    ox, oy, oz = _geodetic_to_ecef(origin_lat, origin_lon, 0.0)
    dx, dy, dz = x - ox, y - oy, z - oz
    lat, lon = math.radians(origin_lat), math.radians(origin_lon)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    c, s = math.cos(angle), math.sin(angle)
    return (x0 + scale * (c * east - s * north),
            y0 + scale * (s * east + c * north))


def json_tang_to_enu(tang: float, angle: float = JSON_ANGLE) -> float:

    return _wrap(tang - angle)


def utm_to_geodetic(easting: float, northing: float, zone: int,
                    northern: bool = True) -> tuple[float, float]:

    n = _WGS84_F / (2.0 - _WGS84_F)
    a_rect = _WGS84_A / (1.0 + n) * (1.0 + n**2 / 4.0 + n**4 / 64.0)
    beta = (n / 2.0 - 2.0 * n**2 / 3.0 + 37.0 * n**3 / 96.0,
            n**2 / 48.0 + n**3 / 15.0,
            17.0 * n**3 / 480.0)
    delta = (2.0 * n - 2.0 * n**2 / 3.0 - 2.0 * n**3,
             7.0 * n**2 / 3.0 - 8.0 * n**3 / 5.0,
             56.0 * n**3 / 15.0)
    xi = (northing - (0.0 if northern else 10_000_000.0)) / (_UTM_K0 * a_rect)
    eta = (easting - 500_000.0) / (_UTM_K0 * a_rect)
    xi_p = xi - sum(b * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
                    for j, b in enumerate(beta, 1))
    eta_p = eta - sum(b * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
                      for j, b in enumerate(beta, 1))
    chi = math.asin(math.sin(xi_p) / math.cosh(eta_p))
    lat = chi + sum(d * math.sin(2 * j * chi) for j, d in enumerate(delta, 1))
    lon0 = math.radians(zone * 6 - 183)
    lon = lon0 + math.atan(math.sinh(eta_p) / math.cos(xi_p))
    return math.degrees(lat), math.degrees(lon)


def _wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


# маршрут
@dataclass
class Route:
    name: str
    lat: np.ndarray
    lon: np.ndarray
    alt: np.ndarray
    yaw: np.ndarray
    curv: np.ndarray
    xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    json_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))

    @property
    def length(self) -> float:
        return float(self.s[-1]) if len(self.s) else 0.0

    def set_frame(self, to_enu) -> None:
        pts = [to_enu(la, lo, al)[:2] for la, lo, al in zip(self.lat, self.lon, self.alt)]
        self.xy = np.asarray(pts, dtype=float)
        seg = np.hypot(np.diff(self.xy[:, 0]), np.diff(self.xy[:, 1]))
        self.s = np.concatenate([[0.0], np.cumsum(seg)])

    def project(self, x: float, y: float) -> tuple[float, float, float, float, float]:

        d = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)
        i = int(np.argmin(d))

        best = (float(self.s[i]), float(d[i]), float(self.xy[i, 0]), float(self.xy[i, 1]))
        for a in (i - 1, i):
            if a < 0 or a + 1 >= len(self.xy):
                continue
            p, q = self.xy[a], self.xy[a + 1]
            seg = q - p
            l2 = float(seg @ seg)
            if l2 < 1e-9:
                continue
            u = min(max(float(((x - p[0]) * seg[0] + (y - p[1]) * seg[1]) / l2), 0.0), 1.0)
            proj = p + u * seg
            dist = math.hypot(x - proj[0], y - proj[1])
            if dist < best[1]:
                best = (float(self.s[a] + u * math.sqrt(l2)), dist, float(proj[0]), float(proj[1]))
        return best[0], best[1], float(self.yaw[i]), best[2], best[3]

    def pose_at(self, s: float) -> tuple[float, float, float, float]:

        if s <= 0.0:
            yaw = float(self.yaw[0])
            return (float(self.xy[0, 0] + s * math.cos(yaw)),
                    float(self.xy[0, 1] + s * math.sin(yaw)), yaw, 0.0)
        if s >= self.length:
            yaw = float(self.yaw[-1])
            over = s - self.length
            return (float(self.xy[-1, 0] + over * math.cos(yaw)),
                    float(self.xy[-1, 1] + over * math.sin(yaw)), yaw, 0.0)
        i = int(np.searchsorted(self.s, s, side="right")) - 1
        u = (s - self.s[i]) / max(self.s[i + 1] - self.s[i], 1e-9)
        x = float(self.xy[i, 0] + u * (self.xy[i + 1, 0] - self.xy[i, 0]))
        y = float(self.xy[i, 1] + u * (self.xy[i + 1, 1] - self.xy[i, 1]))
        yaw = float(self.yaw[i] + u * _wrap(self.yaw[i + 1] - self.yaw[i]))
        curv = float(self.curv[i] + u * (self.curv[i + 1] - self.curv[i]))
        return x, y, _wrap(yaw), curv

    def altitude_at(self, s: float) -> float:
        """JSON z на той же дуге, что и pose_at."""
        n = len(self.alt)
        if n == 0:
            return 0.0
        if n == 1 or len(self.s) < 2 or s <= 0.0:
            return float(self.alt[0])
        if s >= self.length:
            return float(self.alt[-1])
        i = int(np.searchsorted(self.s, s, side="right")) - 1
        i = min(max(i, 0), n - 2)
        span = float(self.s[i + 1] - self.s[i])
        u = 0.0 if span < 1e-9 else (s - float(self.s[i])) / span
        return float(self.alt[i] + u * (self.alt[i + 1] - self.alt[i]))


class RouteMap:
    def __init__(self, files: list[str],
                 origin_lat: float = JSON_ORIGIN_LAT,
                 origin_lon: float = JSON_ORIGIN_LON,
                 x0: float = JSON_X0, y0: float = JSON_Y0,
                 angle: float = JSON_ANGLE, scale: float = JSON_SCALE):
        self.routes: list[Route] = []
        self.angle = angle
        for f in files:
            path = Path(f)
            with path.open(encoding="utf-8") as stream:
                data = json.load(stream)
            pts = data["points"]
            order = data.get("paths", [{}])[0].get("point_indices") or range(len(pts))
            lat, lon, alt, yaw, curv, json_xy = [], [], [], [], [], []
            for i in order:
                p = pts[i]
                json_xy.append((p["x"], p["y"]))
                la, lo = json_xy_to_geodetic(p["x"], p["y"], origin_lat, origin_lon,
                                             x0, y0, angle, scale)
                lat.append(la)
                lon.append(lo)
                alt.append(p.get("z", 0.0))
                yaw.append(json_tang_to_enu(p["tang"], angle))
                curv.append(p.get("curv", 0.0) * scale)
            self.routes.append(Route(path.stem, np.array(lat), np.array(lon), np.array(alt),
                                     np.array(yaw), np.array(curv),
                                     json_xy=np.array(json_xy)))
        self.ready = False

    def set_frame(self, to_enu) -> None:
        for r in self.routes:
            r.set_frame(to_enu)
        self.ready = True

    def nearest(self, x: float, y: float):

        best = None
        for r in self.routes:
            if len(r.xy) == 0:
                continue
            s, lateral, route_yaw, _, _ = r.project(x, y)
            if best is None or lateral < best[2]:
                best = (r, s, lateral, route_yaw)
        return best

    def match(self, x: float, y: float, yaws,
              max_lateral: float, max_heading: float, end_margin_m: float = 50.0):

        if isinstance(yaws, (int, float)):
            yaws = (float(yaws),)
        best = None
        for r in self.routes:
            s, lateral, route_yaw, _, _ = r.project(x, y)
            if lateral > max_lateral:
                continue
            dyaw = min(abs(_wrap(route_yaw - yaw)) for yaw in yaws)
            if dyaw > max_heading:
                continue
            at_end = r.length - s < end_margin_m
            score = lateral / max_lateral + dyaw / max_heading + (10.0 if at_end else 0.0)
            if best is None or score < best[0]:
                best = (score, r, s, lateral, dyaw)
        return None if best is None else best[1:]


@dataclass
class FollowerParams:
    max_lateral_m: float = 15.0
    lock_max_lateral_m: float = 3.0
    max_heading_rad: float = math.radians(35.0)
    min_match_count: int = 10
    min_speed_mps: float = 0.5
    dr_match_step_m: float = 2.0
    switch_radius_m: float = 60.0


# следование
class RouteFollower:

    def __init__(self, route_map: RouteMap, params: FollowerParams):
        self.map = route_map
        self.p = params
        self.route: Route | None = None
        self.s = 0.0
        self.x = self.y = self.z = self.yaw = 0.0
        self.curv = 0.0
        self.initialized = False
        self._match_state: dict[str, tuple] = {}
        self._since_dr_match = 0.0
        self._stopped_at_end = False
        self.distance = 0.0

    @property
    def locked(self) -> bool:
        return self.route is not None

    def init_pose(self, x: float, y: float, yaw: float) -> None:
        self.x, self.y, self.yaw = x, y, yaw
        self.initialized = True

    def set_prelock_pose(self, x: float, y: float, yaw: float) -> None:

        if not self.locked:
            self.x, self.y, self.yaw = x, y, yaw
            self.initialized = True

    def try_match(self, x: float, y: float, yaws, speed: float,
                  require_motion: bool = True) -> bool:

        if self.locked or not self.map.ready:
            return False
        if require_motion and speed < self.p.min_speed_mps:
            return False
        m = self.map.match(x, y, yaws, self.p.max_lateral_m, self.p.max_heading_rad,
                           self.p.switch_radius_m)

        key = "dr" if require_motion else "gnss"
        route_prev, count = self._match_state.get(key, (None, 0))
        if m is None or (route_prev is not None and m[0] is not route_prev):
            self._match_state[key] = (None if m is None else m[0], 0 if m is None else 1)
            return False
        count += 1
        self._match_state[key] = (m[0], count)

        if count < self.p.min_match_count or m[2] > self.p.lock_max_lateral_m:
            return False
        self.route, self.s = m[0], m[1]
        self._set_pose_from_route()
        self.initialized = True
        return True

    def advance(self, ds: float, speed: float) -> None:
        if not self.initialized:
            return
        self.distance += ds
        if self.route is None:
            self.x += ds * math.cos(self.yaw)
            self.y += ds * math.sin(self.yaw)
            self._since_dr_match += ds
            if self.map.ready and self._since_dr_match >= self.p.dr_match_step_m:
                self._since_dr_match = 0.0
                self.try_match(self.x, self.y, self.yaw, speed, require_motion=True)
            return

        self.s += ds
        if self.s >= self.route.length:

            overrun = self.s - self.route.length
            if speed < 0.1:
                self._stopped_at_end = True
            elif (self._stopped_at_end or overrun > self.p.switch_radius_m) \
                    and self._switch_route():
                return
            self.s = min(self.s, self.route.length)
        self._set_pose_from_route()

    def _switch_route(self) -> bool:
        ex, ey = self.route.xy[-1]
        for r in self.map.routes:
            if r is self.route:
                continue
            sx, sy = r.xy[0]
            if math.hypot(sx - ex, sy - ey) <= self.p.switch_radius_m:
                self.route, self.s = r, 0.0
                self._stopped_at_end = False
                self._set_pose_from_route()
                return True
        return False

    def _set_pose_from_route(self) -> None:
        self.x, self.y, self.yaw, self.curv = self.route.pose_at(self.s)
        self.z = self.route.altitude_at(self.s)
