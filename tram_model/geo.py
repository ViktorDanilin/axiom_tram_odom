import math

from tram_model.route_map import enu_to_geodetic

_WGS84_A = 6378137.0
_WGS84_E2 = 6.69437999014e-3


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float):
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    n = _WGS84_A / math.sqrt(1.0 - _WGS84_E2 * sin_lat**2)
    x = (n + alt_m) * cos_lat * math.cos(lon)
    y = (n + alt_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - _WGS84_E2) + alt_m) * sin_lat
    return x, y, z


class LocalEnu:

    def __init__(self, lat_deg: float, lon_deg: float, alt_m: float):
        self._lat_deg, self._lon_deg, self._alt_m = lat_deg, lon_deg, alt_m
        self._origin = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
        lat, lon = math.radians(lat_deg), math.radians(lon_deg)
        self._sl, self._cl = math.sin(lat), math.cos(lat)
        self._so, self._co = math.sin(lon), math.cos(lon)

    def to_enu(self, lat_deg: float, lon_deg: float, alt_m: float):
        x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
        dx, dy, dz = (x - self._origin[0], y - self._origin[1], z - self._origin[2])
        e = -self._so * dx + self._co * dy
        n = -self._sl * self._co * dx - self._sl * self._so * dy + self._cl * dz
        u = self._cl * self._co * dx + self._cl * self._so * dy + self._sl * dz
        return e, n, u

    def to_geodetic(self, east: float, north: float, up: float = 0.0):
        return enu_to_geodetic(
            east, north, up, self._lat_deg, self._lon_deg, self._alt_m)
