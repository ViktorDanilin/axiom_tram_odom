"""Сборка продолженных JSON-путей из выбранных GPS-проездов."""

import copy
import json
import math
from bisect import bisect_left, bisect_right
from pathlib import Path

from .coordinates import gps_to_json_xy
from .gps_tracks import load_gps_track


def _selected_candidates(ids, candidates, side):
    if not ids:
        raise ValueError(f"Для {side} не выбрано ни одного GPS-трека")
    if len(ids) != len(set(ids)):
        raise ValueError(f"В списке {side} повторяется ID")
    by_id = {candidate["id"]: candidate for candidate in candidates}
    missing = set(ids) - by_id.keys()
    if missing:
        raise ValueError(f"Для {side} нет кандидатов: {', '.join(sorted(missing))}")
    return [by_id[bag_id] for bag_id in ids]


def _outward_track(candidate, side, data_dir, junction):
    track = load_gps_track(data_dir, candidate["id"], max_points=None, max_speed_m_s=30)
    xy = [gps_to_json_xy(*gps) for gps in track["points"]]
    sampled = candidate["full_track"]["points"]
    crossing_sample = (
        candidate["extension_track"]["points"][-1] if side == "start"
        else candidate["extension_track"]["points"][0]
    )
    sampled_index = min(range(len(sampled)), key=lambda i: math.dist(sampled[i], crossing_sample))
    expected_index = round(sampled_index * (len(xy) - 1) / (len(sampled) - 1))
    crossing_xy = gps_to_json_xy(*crossing_sample)
    crossing_index = min(
        range(len(xy)),
        key=lambda i: (math.dist(xy[i], crossing_xy), abs(i - expected_index)),
    )
    indices = (range(crossing_index, -1, -1) if side == "start"
               else range(crossing_index, len(xy)))
    z_at_junction = track["altitudes"][crossing_index]
    if not math.isfinite(z_at_junction):
        raise ValueError(f"Нет GPS-высоты в точке стыка у {candidate['id']}")
    dx, dy = junction["x"] - xy[crossing_index][0], junction["y"] - xy[crossing_index][1]
    dz = junction["z"] - z_at_junction
    xyz = []
    distances = []
    for i in indices:
        x, y = xy[i]
        z = track["altitudes"][i]
        if not math.isfinite(z):
            continue
        point = (x, y, z + dz)
        if xyz and math.dist(point[:2], xyz[-1][:2]) < 1e-6:
            continue
        distances.append((distances[-1] if distances else 0.0)
                         + (math.dist(point[:2], xyz[-1][:2]) if xyz else 0.0))
        xyz.append(point)
    if len(xyz) < 2:
        raise ValueError(f"Недостаточно точек продолжения у {candidate['id']}")
    # Один перенос совмещает начало GPS-продолжения со стыком JSON,
    # сохраняя исходное направление и форму всего трека.
    joined = [(x + dx, y + dy, z) for x, y, z in xyz]
    return {
        "id": candidate["id"], "xyz": joined, "distances": distances,
        "length_m": distances[-1],
    }


def _interpolate(track, distance):
    distances, xyz = track["distances"], track["xyz"]
    if distance >= distances[-1]:
        return xyz[-1]
    i = bisect_right(distances, distance)
    if i == 0:
        return xyz[0]
    fraction = (distance - distances[i - 1]) / (distances[i] - distances[i - 1])
    return tuple(a + fraction * (b - a) for a, b in zip(xyz[i - 1], xyz[i]))


def _smooth_extension(points, distances, radius_m):
    """Убрать мелкие GPS-колебания, не сдвигая стык и конец участка."""
    if radius_m <= 0:
        return points
    sigma = radius_m / 2
    smoothed = []
    for i, distance in enumerate(distances):
        left = bisect_left(distances, distance - radius_m)
        right = bisect_right(distances, distance + radius_m)
        neighbors = [
            (points[j], math.exp(-0.5 * ((distances[j] - distance) / sigma) ** 2))
            for j in range(left, right)
        ]
        weight_sum = sum(weight for _, weight in neighbors)
        smoothed.append(tuple(
            sum(point[axis] * weight for point, weight in neighbors) / weight_sum
            for axis in range(3)
        ))
    # Коррекция края сохраняет точный стык, не возвращая шум исходных
    # точек в последние метры участка.
    start_offset = tuple(a - b for a, b in zip(points[0], smoothed[0]))
    end_offset = tuple(a - b for a, b in zip(points[-1], smoothed[-1]))
    corrected = [
        tuple(
            point[axis]
            + start_offset[axis] * max(0.0, 1 - distance / radius_m)
            + end_offset[axis] * max(0.0, 1 - (distances[-1] - distance) / radius_m)
            for axis in range(3)
        )
        for point, distance in zip(smoothed, distances)
    ]
    corrected[0], corrected[-1] = points[0], points[-1]
    return corrected


def _average_extensions(tracks, smoothing_radius_m):
    reference = max(tracks, key=lambda track: track["length_m"])
    # Шаг JSON-пути составляет около метра; сохраняем форму самого длинного
    # проезда на той же шкале, а короткие проезды не экстраполируем.
    samples = list(range(math.ceil(reference["length_m"])))
    if not samples or samples[-1] != reference["length_m"]:
        samples.append(reference["length_m"])
    reference_points = [_interpolate(reference, distance) for distance in samples]
    residuals = []
    fade_length_m = 25
    for distance, reference_point in zip(samples, reference_points):
        weighted_residuals = []
        for track in tracks:
            if track is reference or distance > track["length_m"]:
                continue
            remaining = track["length_m"] - distance
            progress = min(1.0, remaining / fade_length_m)
            weight = (1 - math.cos(math.pi * progress)) / 2
            if weight > 0:
                point = _interpolate(track, distance)
                weighted_residuals.append((
                    tuple(point[axis] - reference_point[axis] for axis in range(3)), weight,
                ))
        total_weight = 1 + sum(weight for _, weight in weighted_residuals)
        residuals.append(tuple(
            sum(point[axis] * weight for point, weight in weighted_residuals) / total_weight
            for axis in range(3)
        ))
    smoothed_residuals = _smooth_extension(residuals, samples, smoothing_radius_m)
    return [
        tuple(point[axis] + residual[axis] for axis in range(3))
        for point, residual in zip(reference_points, smoothed_residuals)
    ], reference


def _make_points(xyz, original_count, start_count):
    points = []
    for i, (x, y, z) in enumerate(xyz):
        before = xyz[max(0, i - 1)]
        after = xyz[min(len(xyz) - 1, i + 1)]
        tangent = math.atan2(after[1] - before[1], after[0] - before[0])
        if 0 < i < len(xyz) - 1:
            heading_before = math.atan2(y - before[1], x - before[0])
            heading_after = math.atan2(after[1] - y, after[0] - x)
            turn = math.atan2(math.sin(heading_after - heading_before),
                              math.cos(heading_after - heading_before))
            span = (math.dist(before[:2], (x, y)) + math.dist((x, y), after[:2])) / 2
            curvature = turn / span if span else 0.0
        else:
            curvature = 0.0
        points.append({"x": x, "y": y, "z": z, "tang": tangent, "curv": curvature})
    return points[:start_count], points[start_count + original_count:]


def build_continued_graph(
    graph_file, data_dir, start_ids, end_ids, candidate_sides, *, smoothing_radius_m=20,
):
    """Продлить единственный путь JSON; вернуть граф и сведения о выборе."""
    if smoothing_radius_m < 0:
        raise ValueError("Радиус сглаживания не может быть отрицательным")
    graph = json.loads(Path(graph_file).read_text())
    if len(graph["paths"]) != 1:
        raise ValueError("Продление поддерживает JSON с одним путём")
    indices = graph["paths"][0]["point_indices"]
    if len(indices) < 2:
        raise ValueError("В JSON-пути меньше двух точек")
    original = [graph["points"][i] for i in indices]
    result = {}
    for side, ids, junction in (
        ("start", start_ids, original[0]), ("end", end_ids, original[-1]),
    ):
        selected = _selected_candidates(ids, candidate_sides[side]["candidates"], side)
        tracks = [_outward_track(candidate, side, data_dir, junction) for candidate in selected]
        result[side], reference = _average_extensions(tracks, smoothing_radius_m)
        result[f"{side}_reference"] = reference["id"]
    start_xyz = list(reversed(result["start"][1:]))
    end_xyz = result["end"][1:]
    all_xyz = (start_xyz + [(p["x"], p["y"], p["z"]) for p in original] + end_xyz)
    start_points, end_points = _make_points(all_xyz, len(original), len(start_xyz))

    continued = copy.deepcopy(graph)
    continued["points"] = start_points + graph["points"] + end_points
    offset = len(start_points)
    continued["paths"][0]["point_indices"] = (
        list(range(offset))
        + [i + offset for i in indices]
        + list(range(offset + len(graph["points"]), len(continued["points"])))
    )
    return continued, {
        "start_reference": result["start_reference"],
        "end_reference": result["end_reference"],
        "added_start": len(start_points),
        "added_end": len(end_points),
    }


def save_continued_graph(
    graph_file, data_dir, output_file, start_ids, end_ids, candidate_sides, *, smoothing_radius_m=20,
):
    graph, info = build_continued_graph(
        graph_file, data_dir, start_ids, end_ids, candidate_sides,
        smoothing_radius_m=smoothing_radius_m,
    )
    output_file = Path(output_file)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text(json.dumps(graph, ensure_ascii=False, indent=2) + "\n")
    return info
