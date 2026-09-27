"""Чтение GPS-треков из ROS 2 bag-файлов SQLite."""

import math
import sqlite3
import struct
from pathlib import Path


TOPIC = "/sensing/gnss/base_link/fix"
EARTH_RADIUS_M = 6_371_000


class NoGpsDataError(ValueError):
    """В bag-файле нет валидных GPS-точек ну1жного топика."""


def _align_cdr(offset, boundary):
    # Выравнивание CDR отсчитывается от конца 4-байтового заголовка.
    return offset + (-(offset - 4) % boundary)


def read_navsatfix(data):
    """Извлечь status, latitude и longitude из little-endian CDR NavSatFix."""
    status, latitude, longitude, _ = _read_navsatfix_with_altitude(data)
    return status, latitude, longitude


def _read_navsatfix_with_altitude(data):
    """Извлечь координаты и высоту из NavSatFix."""
    if len(data) < 48 or data[:2] != b"\x00\x01":
        raise ValueError("Ожидалось сообщение NavSatFix в little-endian CDR")

    frame_length = struct.unpack_from("<I", data, 12)[0]
    offset = 16 + frame_length
    if offset + 4 > len(data):
        raise ValueError("Повреждённая длина header.frame_id")
    status = struct.unpack_from("<b", data, offset)[0]
    offset = _align_cdr(offset + 1, 2) + 2  # status.service (uint16)
    offset = _align_cdr(offset, 8)
    latitude, longitude, altitude = struct.unpack_from("<ddd", data, offset)
    return status, latitude, longitude, altitude


def _distance_m(first, second):
    """Расстояние по сфере между двумя [latitude, longitude] в метрах."""
    lat1, lon1 = map(math.radians, first)
    lat2, lon2 = map(math.radians, second)
    dlat, dlon = lat2 - lat1, lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def load_gps_track(data_dir, bag_id, *, max_points=1_200, max_speed_m_s=None):
    """Вернуть трек, длину полного пути и число точек до прореживания.

    При max_speed_m_s точки с неправдоподобным скачком от последней
    принятой точки пропускаются до вычисления длины и прореживания.
    """
    bag_files = sorted((Path(data_dir) / bag_id).glob("*.db3"))
    if not bag_files:
        raise FileNotFoundError(f"Не найден .db3 для {bag_id}")
    if max_points is not None and max_points < 2:
        raise ValueError("max_points должен быть не меньше 2")
    if max_speed_m_s is not None and max_speed_m_s <= 0:
        raise ValueError("max_speed_m_s должен быть положительным")

    samples = []
    for bag_file in bag_files:
        with sqlite3.connect(bag_file.resolve().as_uri() + "?mode=ro", uri=True) as db:
            topic = db.execute("SELECT id FROM topics WHERE name = ?", (TOPIC,)).fetchone()
            if topic is None:
                continue
            for timestamp, data in db.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (topic[0],),
            ):
                status, lat, lon, altitude = _read_navsatfix_with_altitude(data)
                if (
                    status >= 0
                    and math.isfinite(lat)
                    and math.isfinite(lon)
                    and -90 <= lat <= 90
                    and -180 <= lon <= 180
                    and (lat, lon) != (0.0, 0.0)
                ):
                    samples.append((timestamp, [lat, lon], altitude))

    if not samples:
        raise NoGpsDataError(f"В {bag_id} нет валидных GPS-координат для {TOPIC}")
    samples.sort(key=lambda sample: sample[0])

    accepted = [samples[0]]
    length_m = 0.0
    for timestamp, point, altitude in samples[1:]:
        prev_time, prev_point, _ = accepted[-1]
        elapsed_s = (timestamp - prev_time) / 1_000_000_000
        if elapsed_s <= 0:
            continue
        segment_m = _distance_m(prev_point, point)
        if max_speed_m_s is not None and segment_m / elapsed_s > max_speed_m_s:
            continue
        accepted.append((timestamp, point, altitude))
        length_m += segment_m

    points = [point for _, point, _ in accepted]
    altitudes = [altitude for _, _, altitude in accepted]
    point_count = len(points)
    if max_points is not None and point_count > max_points:
        indices = [round(i * (point_count - 1) / (max_points - 1)) for i in range(max_points)]
        points = [points[index] for index in indices]
        altitudes = [altitudes[index] for index in indices]

    return {
        "id": bag_id, "points": points, "altitudes": altitudes,
        "point_count": point_count, "length_m": length_m,
    }
