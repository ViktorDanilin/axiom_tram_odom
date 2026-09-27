"""Fuse two GNSS antennas into one base_link fix in each ROS 2 SQLite bag.

No ROS installation or third-party Python packages are required. The source
bags are left untouched. Messages without a synchronized, plausible pair are
not extrapolated into a base_link fix.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
import math
from pathlib import Path
import re
import shutil
import sqlite3
import struct
import tempfile


MASTER = "/sensing/gnss/master/fix"
ROVER = "/sensing/gnss/rover/fix"
OLD_GNSS = (MASTER, "/sensing/gnss/master/vel", ROVER, "/sensing/gnss/rover/vel")
UNION = "/sensing/gnss/base_link/fix"
MASTER_X = -9.873
ROVER_X = 2.563
ANTENNA_Z = 3.0
BASELINE = ROVER_X - MASTER_X
NS = 1_000_000_000
WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


@dataclass(frozen=True)
class Fix:
    bag_time: int
    file_name: str
    latitude: float
    longitude: float
    altitude: float
    status: int
    service: int


def align(value: int, boundary: int) -> int:
    return (value + boundary - 1) & ~(boundary - 1)


def decode_fix(data: bytes, bag_time: int, file_name: str) -> Fix | None:
    """Decode the relevant fields of a CDR sensor_msgs/msg/NavSatFix."""
    if len(data) < 52 or data[:2] not in (b"\x00\x01", b"\x00\x00"):
        raise ValueError("Invalid CDR NavSatFix")
    order = "<" if data[1] == 1 else ">"
    sec, nsec, frame_len = struct.unpack_from(order + "iII", data, 4)
    if not 0 <= nsec < NS or not 1 <= frame_len <= len(data) - 16:
        raise ValueError("Invalid NavSatFix header")
    status_offset = 16 + frame_len
    service_offset = 4 + align(status_offset + 1 - 4, 2)
    coordinate_offset = 4 + align(service_offset + 2 - 4, 8)
    if len(data) < coordinate_offset + 24 + 72 + 1:
        raise ValueError("Truncated NavSatFix")
    status = struct.unpack_from("b", data, status_offset)[0]
    service = struct.unpack_from(order + "H", data, service_offset)[0]
    latitude, longitude, altitude = struct.unpack_from(order + "ddd", data, coordinate_offset)
    if status < 0 or not (math.isfinite(latitude) and math.isfinite(longitude)
                          and math.isfinite(altitude)) or not -90 <= latitude <= 90 \
            or not -180 <= longitude <= 180 or (latitude == 0 and longitude == 0):
        return None
    return Fix(bag_time, file_name, latitude, longitude, altitude, status, service)


def ecef(fix: Fix) -> tuple[float, float, float]:
    lat, lon = math.radians(fix.latitude), math.radians(fix.longitude)
    s = math.sin(lat)
    radius = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)
    return ((radius + fix.altitude) * math.cos(lat) * math.cos(lon),
            (radius + fix.altitude) * math.cos(lat) * math.sin(lon),
            (radius * (1 - WGS84_E2) + fix.altitude) * s)


def geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert ECEF back to WGS84 using a short fixed-point iteration."""
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1 - WGS84_E2))
    for _ in range(7):
        s = math.sin(lat)
        radius = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)
        height = p / math.cos(lat) - radius
        lat = math.atan2(z, p * (1 - WGS84_E2 * radius / (radius + height)))
    s = math.sin(lat)
    radius = WGS84_A / math.sqrt(1 - WGS84_E2 * s * s)
    return math.degrees(lat), math.degrees(lon), p / math.cos(lat) - radius


def fuse(master: Fix, rover: Fix, baseline_tolerance: float) -> tuple[float, float, float] | None:
    """Estimate heading from the antenna baseline and shift to base_link."""
    mx, my, mz = ecef(master)
    rx, ry, rz = ecef(rover)
    dx, dy, dz = rx - mx, ry - my, rz - mz
    lat, lon = math.radians(master.latitude), math.radians(master.longitude)
    east = -math.sin(lon) * dx + math.cos(lon) * dy
    north = (-math.sin(lat) * math.cos(lon) * dx
             - math.sin(lat) * math.sin(lon) * dy + math.cos(lat) * dz)
    horizontal = math.hypot(east, north)
    if abs(horizontal - BASELINE) > baseline_tolerance:
        return None
    # Each antenna yields a base_link estimate. Average the two after moving
    # them along the measured vehicle x axis; both antennas are 3 m above it.
    base_east = 0.5 * east - 0.5 * (MASTER_X + ROVER_X) * east / horizontal
    base_north = 0.5 * north - 0.5 * (MASTER_X + ROVER_X) * north / horizontal
    up = (math.cos(lat) * math.cos(lon) * dx
          + math.cos(lat) * math.sin(lon) * dy + math.sin(lat) * dz)
    base_up = 0.5 * up - ANTENNA_Z
    ex = -math.sin(lon) * base_east - math.sin(lat) * math.cos(lon) * base_north \
         + math.cos(lat) * math.cos(lon) * base_up
    ey = math.cos(lon) * base_east - math.sin(lat) * math.sin(lon) * base_north \
         + math.cos(lat) * math.sin(lon) * base_up
    ez = math.cos(lat) * base_north + math.sin(lat) * base_up
    return geodetic(mx + ex, my + ey, mz + ez)


def encode_fix(timestamp: int, position: tuple[float, float, float],
               status: int, service: int) -> bytes:
    """Write a little-endian CDR NavSatFix with unknown covariance."""
    sec, nsec = divmod(timestamp, NS)
    data = bytearray(b"\x00\x01\x00\x00")

    def add(fmt: str, alignment: int, *values: object) -> None:
        data.extend(b"\x00" * (align(len(data) - 4, alignment) - (len(data) - 4)))
        data.extend(struct.pack("<" + fmt, *values))

    add("iI", 4, sec, nsec)
    frame = b"base_link\x00"
    add("I", 4, len(frame))
    data.extend(frame)
    add("b", 1, status)
    add("H", 2, service)
    add("ddd", 8, *position)
    add("9d", 8, *([0.0] * 9))
    add("B", 1, 0)  # COVARIANCE_TYPE_UNKNOWN: the inputs have no covariance.
    return bytes(data)


def pair_fixes(masters: list[Fix], rovers: list[Fix], max_dt_ns: int):
    """Match each fix at most once, choosing the nearest available timestamp."""
    i = j = 0
    while i < len(masters) and j < len(rovers):
        m, r = masters[i], rovers[j]
        delta = r.bag_time - m.bag_time
        if delta > max_dt_ns:
            i += 1
        elif delta < -max_dt_ns:
            j += 1
        elif i + 1 < len(masters) and abs(r.bag_time - masters[i + 1].bag_time) < abs(delta):
            i += 1
        elif j + 1 < len(rovers) and abs(rovers[j + 1].bag_time - m.bag_time) < abs(delta):
            j += 1
        else:
            yield m, r
            i += 1
            j += 1


def rewrite_metadata(source: str, counts: Counter[str],
                     file_stats: dict[str, tuple[int | None, int | None, int]]) -> str:
    """Keep the source QoS and layout while replacing GNSS topic entries."""
    lines = source.splitlines(keepends=True)
    begin = next(i for i, line in enumerate(lines) if line.startswith("  topics_with_message_count:"))
    end = next(i for i in range(begin + 1, len(lines)) if lines[i].startswith("  compression_format:"))
    topic_end = end
    blocks = []
    block = []
    for line in lines[begin + 1:topic_end]:
        if line.startswith("    - topic_metadata:"):
            if block:
                blocks.append(block)
            block = [line]
        else:
            block.append(line)
    if block:
        blocks.append(block)

    def name_of(block: list[str]) -> str:
        return next(line.split(": ", 1)[1].strip() for line in block if line.startswith("        name: "))

    master_block = next(block for block in blocks if name_of(block) == MASTER)
    output_topics = []
    for index, block in enumerate(blocks + [master_block]):
        name = name_of(block)
        if index < len(blocks) and name in OLD_GNSS:
            continue
        target = UNION if index == len(blocks) else name
        for line in block:
            if line.startswith("        name: "):
                line = line.replace(name, target, 1)
            elif line.startswith("      message_count: "):
                line = re.sub(r"\d+(?=\s*$)", str(counts[target]), line)
            output_topics.append(line)
    lines[begin + 1:topic_end] = output_topics

    starts = [start for start, _, count in file_stats.values() if count and start is not None]
    ends = [end for _, end, count in file_stats.values() if count and end is not None]
    first, last, total = min(starts), max(ends), sum(row[2] for row in file_stats.values())
    context = section = file_name = None
    result = []
    for line in lines:
        if line.startswith("  files:"):
            section = "files"
        elif line.startswith("  compression_format:"):
            section = None
        if line.startswith("  duration:"):
            context = "global_duration"
        elif line.startswith("  starting_time:"):
            context = "global_start"
        elif section == "files" and line.startswith("      duration:"):
            context = "file_duration"
        elif section == "files" and line.startswith("      starting_time:"):
            context = "file_start"
        if section == "files" and line.startswith("    - path: "):
            file_name = line.split(": ", 1)[1].strip()
        if line.startswith("    nanoseconds:") and context == "global_duration":
            line = re.sub(r"\d+(?=\s*$)", str(last - first), line)
        elif line.startswith("    nanoseconds_since_epoch:") and context == "global_start":
            line = re.sub(r"\d+(?=\s*$)", str(first), line)
        elif section == "files" and file_name:
            start, finish, count = file_stats[file_name]
            if line.startswith("        nanoseconds_since_epoch:") and context == "file_start":
                line = re.sub(r"\d+(?=\s*$)", str(start or 0), line)
            elif line.startswith("        nanoseconds:") and context == "file_duration":
                line = re.sub(r"\d+(?=\s*$)", str(finish - start if count else 0), line)
            elif line.startswith("      message_count: "):
                line = re.sub(r"\d+(?=\s*$)", str(count), line)
        elif line.startswith("  message_count: "):
            line = re.sub(r"\d+(?=\s*$)", str(total), line)
        result.append(line)
    return "".join(result)


def process_bag(source: Path, destination: Path, max_dt_ns: int,
                baseline_tolerance: float) -> tuple[int, int]:
    if destination.exists():
        raise ValueError(f"Output already exists: {destination}")
    db_paths = sorted(source.glob("*.db3"))
    if not db_paths or not (source / "metadata.yaml").is_file():
        raise ValueError(f"Not a ROS 2 SQLite bag: {source}")
    fixes: dict[str, list[Fix]] = {MASTER: [], ROVER: []}
    topic_template = None
    for path in db_paths:
        with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as db:
            for name in (MASTER, ROVER):
                row = db.execute("SELECT id, type, serialization_format, offered_qos_profiles "
                                 "FROM topics WHERE name = ?", (name,)).fetchone()
                if row is None or row[1:3] != ("sensor_msgs/msg/NavSatFix", "cdr"):
                    raise ValueError(f"Missing CDR NavSatFix {name} in {path}")
                if name == MASTER:
                    topic_template = row[1:]
                for bag_time, data in db.execute(
                        "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (row[0],)):
                    fix = decode_fix(data, bag_time, path.name)
                    if fix is not None:
                        fixes[name].append(fix)
    if topic_template is None:
        raise ValueError(f"No master GNSS topic in {source}")
    for points in fixes.values():
        points.sort(key=lambda fix: fix.bag_time)
    fused: dict[str, list[tuple[int, bytes]]] = {path.name: [] for path in db_paths}
    paired = rejected = 0
    for master, rover in pair_fixes(fixes[MASTER], fixes[ROVER], max_dt_ns):
        paired += 1
        position = fuse(master, rover, baseline_tolerance)
        if position is None:
            rejected += 1
            continue
        timestamp = (master.bag_time + rover.bag_time) // 2
        fused[master.file_name].append((timestamp, encode_fix(
            timestamp, position, min(master.status, rover.status), master.service & rover.service)))

    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="gps_union_", dir=destination.parent) as temp:
        stage = Path(temp)
        counts: Counter[str] = Counter()
        file_stats = {}
        for path in db_paths:
            with sqlite3.connect(f"file:{path}?mode=ro", uri=True) as src, \
                    sqlite3.connect(stage / path.name) as db:
                src.backup(db)
                with db:
                    db.execute("DELETE FROM messages WHERE topic_id IN "
                               "(SELECT id FROM topics WHERE name IN (?,?,?,?))", OLD_GNSS)
                    db.execute("DELETE FROM topics WHERE name IN (?,?,?,?)", OLD_GNSS)
                    cursor = db.execute("INSERT INTO topics (name, type, serialization_format, offered_qos_profiles) "
                                        "VALUES (?, ?, ?, ?)", (UNION, *topic_template))
                    db.executemany("INSERT INTO messages (topic_id, timestamp, data) VALUES (?, ?, ?)",
                                   ((cursor.lastrowid, t, blob) for t, blob in fused[path.name]))
                counts.update(dict(db.execute(
                    "SELECT topics.name, COUNT(messages.id) FROM topics "
                    "LEFT JOIN messages ON topics.id = messages.topic_id GROUP BY topics.id")))
                file_stats[path.name] = db.execute(
                    "SELECT MIN(timestamp), MAX(timestamp), COUNT(*) FROM messages").fetchone()
        metadata = rewrite_metadata((source / "metadata.yaml").read_text(encoding="utf-8"),
                                    counts, file_stats)
        (stage / "metadata.yaml").write_text(metadata, encoding="utf-8")
        shutil.move(str(stage), str(destination))
    return sum(map(len, fused.values())), rejected


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("data"), help="Bag directory or parent directory")
    parser.add_argument("--output", type=Path, default=Path("data_gps_union"))
    parser.add_argument("--max-time-diff", type=float, default=0.1, help="Maximum pair time difference, seconds")
    parser.add_argument("--baseline-tolerance", type=float, default=2.0,
                        help="Maximum horizontal baseline error, metres")
    args = parser.parse_args()
    if args.max_time_diff <= 0 or args.baseline_tolerance <= 0:
        parser.error("Time difference and baseline tolerance must be positive")
    if not args.input.is_dir():
        parser.error(f"Input does not exist: {args.input}")
    bags = ([args.input] if (args.input / "metadata.yaml").is_file()
            else sorted(path for path in args.input.iterdir() if (path / "metadata.yaml").is_file()))
    if not bags:
        parser.error(f"No ROS 2 bags in {args.input}")
    total = 0
    for bag in bags:
        count, rejected = process_bag(bag, args.output / bag.name,
                                      round(args.max_time_diff * NS), args.baseline_tolerance)
        total += count
        print(f"{bag.name}: {count} fused fixes, {rejected} rejected baseline pairs", flush=True)
    print(f"Created {len(bags)} bags with {total} base_link fixes in {args.output}")


if __name__ == "__main__":
    main()
