"""Поиск GPS-участков от колец до начала JSON-путей и от их конца до колец."""

import json
import math
from pathlib import Path
from statistics import median

from .coordinates import gps_to_json_xy
from .gps_tracks import NoGpsDataError, load_gps_track


def _inside_point(points, distance_m=200):
    """Точка внутри JSON-пути на заданном расстоянии от его начала."""
    traveled = 0.0
    for first, second in zip(points, points[1:]):
        traveled += math.dist(first, second)
        if traveled >= distance_m:
            return second
    return points[-1]


def _spatial_index(points, cell_size=10):
    """Разложить метровые точки JSON-пути по ячейкам для быстрого поиска."""
    cells = {}
    for point in points:
        cell = (math.floor(point[0] / cell_size), math.floor(point[1] / cell_size))
        cells.setdefault(cell, []).append(point)
    return cells


def _near_known_path(point, cells, tolerance_m=8, cell_size=10):
    x_cell = math.floor(point[0] / cell_size)
    y_cell = math.floor(point[1] / cell_size)
    tolerance_sq = tolerance_m * tolerance_m
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for known in cells.get((x_cell + dx, y_cell + dy), ()):
                if (point[0] - known[0]) ** 2 + (point[1] - known[1]) ** 2 <= tolerance_sq:
                    return True
    return False


def _overlap_length(xy, cells):
    """Измерить длину GPS-линии, идущую не дальше 8 м от JSON-пути."""
    total_m = matched_m = 0.0
    for first, second in zip(xy, xy[1:]):
        segment_m = math.dist(first, second)
        total_m += segment_m
        midpoint = ((first[0] + second[0]) / 2, (first[1] + second[1]) / 2)
        if _near_known_path(midpoint, cells):
            matched_m += segment_m
    return matched_m, total_m


def _graph_junctions(graph_dir):
    junctions = {}
    for graph_file in sorted(Path(graph_dir).glob("*.json")):
        graph = json.loads(graph_file.read_text())
        points = graph["points"]
        for path_number, path in enumerate(graph["paths"], start=1):
            xy = [(points[i]["x"], points[i]["y"]) for i in path["point_indices"]]
            if len(xy) < 2:
                continue
            name = graph_file.name if len(graph["paths"]) == 1 else f"{graph_file.name} · {path_number}"
            junctions[name] = {
                "path_length_m": sum(math.dist(a, b) for a, b in zip(xy, xy[1:])),
                "spatial_index": _spatial_index(xy),
                "start": {"junction_xy": xy[0], "inside_xy": _inside_point(xy)},
                "end": {"junction_xy": xy[-1], "inside_xy": _inside_point(xy[::-1])},
            }
    if not junctions:
        raise ValueError(f"Не найдены пути в {graph_dir}")
    return junctions


def _candidate(track, xy, junction, side, endpoint_tolerance_m,
               inside_tolerance_m, min_extension_m, min_ring_distance_m):
    """Проверить один край JSON-пути с учётом направления GPS-трека."""
    distances = [math.dist(point, junction["junction_xy"]) for point in xy]
    crossing_index = min(range(len(xy)), key=distances.__getitem__)
    junction_gap_m = distances[crossing_index]
    if junction_gap_m > endpoint_tolerance_m:
        return None

    if side == "start":
        if crossing_index == len(xy) - 1:
            return None
        inside_points = xy[crossing_index + 1:]
        extension_xy = xy[:crossing_index + 1]
        extension_points = track["points"][:crossing_index + 1]
        ring_endpoint_xy = xy[0]
    else:
        if crossing_index == 0:
            return None
        inside_points = xy[:crossing_index]
        extension_xy = xy[crossing_index:]
        extension_points = track["points"][crossing_index:]
        ring_endpoint_xy = xy[-1]

    inside_gap_m = min(math.dist(point, junction["inside_xy"]) for point in inside_points)
    if inside_gap_m > inside_tolerance_m:
        return None
    extension_m = sum(math.dist(a, b) for a, b in zip(extension_xy, extension_xy[1:]))
    ring_distance_m = math.dist(ring_endpoint_xy, junction["junction_xy"])
    if extension_m < min_extension_m or ring_distance_m < min_ring_distance_m:
        return None

    return {
        "id": track["id"],
        "junction_gap_m": junction_gap_m,
        "inside_gap_m": inside_gap_m,
        "extension_m": extension_m,
        "ring_distance_m": ring_distance_m,
        "ring_endpoint_xy": ring_endpoint_xy,
        "full_track": {"id": track["id"], "points": track["points"]},
        "extension_track": {"id": track["id"], "points": extension_points},
    }


def find_endpoint_extensions(
    graph_dir,
    data_dir,
    *,
    endpoint_tolerance_m=15,
    inside_tolerance_m=30,
    min_extension_m=120,
    min_ring_distance_m=100,
    ring_cluster_radius_m=80,
    max_speed_m_s=30,
    min_overlap_ratio=0.75,
    min_graph_coverage=0.75,
):
    """Найти продолжения с обеих сторон каждого JSON-пути.

    Для начала GPS должен прийти с кольца к первой точке JSON и пройти
    200 м внутри пути. Для конца — пройти последние 200 м JSON, затем
    уйти от последней точки к кольцу. Район кольца оценивается медианой
    начальных или конечных GPS-точек предварительных кандидатов. Кроме
    стыка, не менее 75% длины GPS-линии должно идти в пределах 8 м от
    известного JSON-пути; геометрия GPS-точек не меняется.
    """
    junctions = _graph_junctions(graph_dir)
    preliminary = {
        name: {side: [] for side in ("start", "end")}
        for name in junctions
    }

    for bag_dir in sorted(Path(data_dir).iterdir()):
        if not bag_dir.is_dir():
            continue
        try:
            track = load_gps_track(data_dir, bag_dir.name, max_points=1_200, max_speed_m_s=max_speed_m_s)
        except NoGpsDataError:
            continue
        xy = [gps_to_json_xy(*point) for point in track["points"]]
        if len(xy) < 2:
            continue
        for name, graph in junctions.items():
            found = []
            for side in ("start", "end"):
                candidate = _candidate(
                    track, xy, graph[side], side, endpoint_tolerance_m,
                    inside_tolerance_m, min_extension_m, min_ring_distance_m,
                )
                if candidate is not None:
                    found.append((side, candidate))
            if not found:
                continue
            matched_m, total_m = _overlap_length(xy, graph["spatial_index"])
            overlap_ratio = matched_m / total_m if total_m else 0.0
            graph_coverage = matched_m / graph["path_length_m"]
            if overlap_ratio < min_overlap_ratio or graph_coverage < min_graph_coverage:
                continue
            for side, candidate in found:
                preliminary[name][side].append({
                    **candidate,
                    "overlap_ratio": overlap_ratio,
                    "graph_coverage": graph_coverage,
                    "matched_m": matched_m,
                })

    results = {}
    for name, graph in junctions.items():
        results[name] = {}
        for side in ("start", "end"):
            junction = graph[side]
            candidates = preliminary[name][side]
            if candidates:
                ring_xy = (
                    median(candidate["ring_endpoint_xy"][0] for candidate in candidates),
                    median(candidate["ring_endpoint_xy"][1] for candidate in candidates),
                )
                candidates = [
                    {**candidate, "ring_gap_m": math.dist(candidate["ring_endpoint_xy"], ring_xy)}
                    for candidate in candidates
                    if math.dist(candidate["ring_endpoint_xy"], ring_xy) <= ring_cluster_radius_m
                ]
                candidates.sort(key=lambda candidate: (-candidate["overlap_ratio"], -candidate["extension_m"]))
            else:
                ring_xy = None
            results[name][side] = {**junction, "ring_xy": ring_xy, "candidates": candidates}
    return results
