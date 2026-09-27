"""Преобразование между GPS и системой координат path_graphs."""

import math

import pymap3d as pm


def gps_to_json_xy(lat, lon):
    east, north, _ = pm.geodetic2enu(
        lat, lon, 0,
        55.8104031450, 37.4623050517, 0,
    )
    angle = -0.021921875
    c, s = math.cos(angle), math.sin(angle)
    x = 103634.578 + 0.999461 * (c * east - s * north)
    y = 86048.242 + 0.999461 * (s * east + c * north)
    return x, y


def json_xy_to_gps(x, y):
    """Обратное преобразование координат JSON для отображения на GPS-карте."""
    angle = -0.021921875
    c, s = math.cos(angle), math.sin(angle)
    dx = (x - 103634.578) / 0.999461
    dy = (y - 86048.242) / 0.999461
    east = c * dx + s * dy
    north = -s * dx + c * dy
    lat, lon, _ = pm.enu2geodetic(
        east, north, 0,
        55.8104031450, 37.4623050517, 0,
    )
    return lat, lon
