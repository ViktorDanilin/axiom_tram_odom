"""Экспорт GPS-измерений из data/*.db3 в JSON с координатами path_graphs.

Запуск из корня проекта: uv run python helpers/gps_to_json.py
Один bag: uv run python helpers/gps_to_json.py 30618_0e41eac3
"""

import argparse
import json
import math
import sqlite3
import sys
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_DIR / "notebooks"))

from tools.coordinates import gps_to_json_xy  # noqa: E402
from tools.gps_tracks import TOPIC, read_navsatfix  # noqa: E402


def convert_bag(bag_dir: Path) -> dict:
    """Прочитать все части bag и вернуть валидные GPS-точки в координатах JSON."""
    bag_files = sorted(bag_dir.glob("*.db3"))
    if not bag_files:
        raise FileNotFoundError(f"В {bag_dir} нет файлов .db3")

    samples = []
    for bag_file in bag_files:
        with sqlite3.connect(bag_file.resolve().as_uri() + "?mode=ro", uri=True) as db:
            topic = db.execute("SELECT id FROM topics WHERE name = ?", (TOPIC,)).fetchone()
            if topic is None:
                continue
            rows = db.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp",
                (topic[0],),
            )
            for timestamp_ns, data in rows:
                status, lat, lon = read_navsatfix(data)
                if (
                    status < 0
                    or not math.isfinite(lat)
                    or not math.isfinite(lon)
                    or not (-90 <= lat <= 90 and -180 <= lon <= 180)
                    or (lat, lon) == (0.0, 0.0)
                ):
                    continue
                x, y = gps_to_json_xy(lat, lon)
                samples.append(
                    {"timestamp_ns": timestamp_ns, "x": x, "y": y, "latitude": lat, "longitude": lon}
                )

    if not samples:
        raise ValueError(f"В {bag_dir} нет валидных GPS-координат для {TOPIC}")
    samples.sort(key=lambda point: point["timestamp_ns"])
    return {"id": bag_dir.name, "topic": TOPIC, "points": samples}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag_ids", nargs="*", help="Имена каталогов bag; по умолчанию все из data/")
    parser.add_argument("--data-dir", type=Path, default=PROJECT_DIR / "data")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_DIR / "gps_json")
    args = parser.parse_args()

    bag_dirs = (
        [args.data_dir / bag_id for bag_id in args.bag_ids]
        if args.bag_ids
        else sorted(path for path in args.data_dir.iterdir() if path.is_dir())
    )
    if not bag_dirs:
        parser.error(f"В {args.data_dir} нет каталогов bag")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    for bag_dir in bag_dirs:
        result = convert_bag(bag_dir)
        output_file = args.output_dir / f"{bag_dir.name}.json"
        output_file.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(f"{output_file}: {len(result['points'])} точек")


if __name__ == "__main__":
    main()
