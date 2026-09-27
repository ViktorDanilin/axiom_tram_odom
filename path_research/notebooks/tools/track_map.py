"""Отображение GPS-треков на ipyleaflet-карте."""

import colorsys
from base64 import b64encode

from IPython.display import display
from ipyleaflet import (
    CircleMarker,
    Icon,
    LayerGroup,
    LayersControl,
    LegendControl,
    Map,
    Marker,
    Polyline,
    basemaps,
)
from ipywidgets import Layout


def _finish_icon(color):
    """Цветная точка с чёрно-белым клетчатым флагом внутри."""
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg" width="28" height="28" viewBox="0 0 28 28">'
        f'<circle cx="14" cy="14" r="12" fill="{color}" stroke="#222" stroke-width="1"/>'
        '<path d="M9 7v15" stroke="#111" stroke-width="1.5"/>'
        '<path d="M10 8h12v9H10z" fill="#fff" stroke="#111" stroke-width="0.7"/>'
        '<path d="M10 8h3v3h-3zM16 8h3v3h-3zM13 11h3v3h-3zM19 11h3v3h-3z'
        'M10 14h3v3h-3zM16 14h3v3h-3z" fill="#111"/>'
        '</svg>'
    )
    return Icon(
        icon_url="data:image/svg+xml;base64," + b64encode(svg.encode()).decode(),
        icon_size=[28, 28],
        icon_anchor=[14, 14],
    )


def show_tracks_map(tracks, height=320, focus_bounds=None):
    """Показать GPS-треки; focus_bounds задаёт область начального просмотра."""
    if not tracks or any(not track["points"] for track in tracks):
        raise ValueError("Для карты нужен хотя бы один непустой GPS-трек")

    coordinates = [point for track in tracks for point in track["points"]]
    south = min(point[0] for point in coordinates)
    west = min(point[1] for point in coordinates)
    north = max(point[0] for point in coordinates)
    east = max(point[1] for point in coordinates)
    view_bounds = focus_bounds if focus_bounds is not None else [[south, west], [north, east]]
    (view_south, view_west), (view_north, view_east) = view_bounds
    map_widget = Map(
        center=((view_south + view_north) / 2, (view_west + view_east) / 2),
        zoom=16 if focus_bounds is not None else 13,
        basemap=basemaps.OpenStreetMap.Mapnik,
        layout=Layout(height=f"{height}px"),
        scroll_wheel_zoom=True,
    )
    legend = {}
    for index, track in enumerate(tracks):
        red, green, blue = colorsys.hsv_to_rgb(index / len(tracks), 0.75, 0.8)
        color = f"#{round(red * 255):02x}{round(green * 255):02x}{round(blue * 255):02x}"
        points = track["points"]
        line = Polyline(locations=points, color=color, weight=4, fill=False)
        start = CircleMarker(location=points[0], radius=6, color=color, fill_color=color, fill_opacity=1)
        end = Marker(location=points[-1], icon=_finish_icon(color), draggable=False, title=f"{track['id']}: финиш")
        map_widget.add(LayerGroup(layers=(line, start, end), name=track["id"]))
        legend[track["id"]] = color

    map_widget.add(LayersControl(position="topright", collapsed=False))
    # map_widget.add(LegendControl(legend, title="Треки", position="bottomright"))
    display(map_widget)
    map_widget.fit_bounds(view_bounds)
    return map_widget
