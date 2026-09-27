"""Загрузка путей из path_graphs в GPS-координатах."""

import json
from pathlib import Path

from .coordinates import json_xy_to_gps


def load_path_graph_tracks(graph_dir):
    """Вернуть пути из всех JSON-файлов как треки для show_tracks_map."""
    graph_files = sorted(Path(graph_dir).glob("*.json"))
    if not graph_files:
        raise FileNotFoundError(f"Не найдены JSON-файлы в {graph_dir}")

    tracks = []
    for graph_file in graph_files:
        graph = json.loads(graph_file.read_text())
        points = graph["points"]
        for index, path in enumerate(graph["paths"], start=1):
            coords = [
                list(json_xy_to_gps(points[i]["x"], points[i]["y"]))
                for i in path["point_indices"]
            ]
            if coords:
                name = graph_file.name if len(graph["paths"]) == 1 else f"{graph_file.name} · {index}"
                tracks.append({"id": name, "points": coords})

    if not tracks:
        raise ValueError(f"Не найдены непустые пути в {graph_dir}")
    return tracks
