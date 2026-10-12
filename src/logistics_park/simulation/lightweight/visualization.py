"""Dependency-free SVG trajectory overview in world coordinates."""
from html import escape
from pathlib import Path

from logistics_park.map.road_graph import RoadGraph
from logistics_park.safety.collision import OBB


def write_svg(graph: RoadGraph, rows: list[dict], length: float, width: float, output: Path,
              title: str = "Minimal closed-loop trajectories (metres)") -> None:
    world_points = [point for edge in graph.edges.values() for point in edge.centerline.points]
    world_points += [(row["x"], row["y"]) for row in rows]
    min_x = min(point[0] for point in world_points) - 0.15
    max_x = max(point[0] for point in world_points) + 0.15
    min_y = min(point[1] for point in world_points) - 0.15
    max_y = max(point[1] for point in world_points) + 0.15
    scale = min(700 / (max_x - min_x), 700 / (max_y - min_y))

    def points(items):
        return " ".join(f"{(x-min_x)*scale+40:.2f},{740-(y-min_y)*scale:.2f}" for x, y in items)

    colors = ["#2563eb", "#dc2626", "#059669", "#9333ea"]
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 800 800">',
             '<rect width="800" height="800" fill="#f8fafc"/>',
             f'<text x="40" y="28" font-family="sans-serif" font-size="18">{escape(title)}</text>']
    for edge in graph.edges.values():
        parts.append(f'<polyline points="{points(edge.centerline.points)}" stroke="#e2e8f0" stroke-width="{edge.width*scale:.2f}" fill="none"/>')
    for edge in graph.edges.values():
        parts.append(f'<polyline points="{points(edge.centerline.points)}" stroke="#64748b" stroke-dasharray="5 5" stroke-width="1" fill="none"/>')
    ids = sorted({row["vehicle_id"] for row in rows})
    for index, vehicle_id in enumerate(ids):
        trace = [row for row in rows if row["vehicle_id"] == vehicle_id]
        color = colors[index % len(colors)]
        parts.append(f'<polyline points="{points([(row["x"],row["y"]) for row in trace])}" stroke="{color}" stroke-width="2" fill="none" opacity="0.75"/>')
        last = trace[-1]
        box = OBB(last["x"], last["y"], last["heading"], length, width)
        parts.append(f'<polygon points="{points(box.corners())}" fill="{color}"/>')
        parts.append(f'<text x="{40+index*175}" y="780" font-family="sans-serif" fill="{color}">{escape(vehicle_id)}</text>')
    parts.append('</svg>')
    output.write_text("\n".join(parts)+"\n", encoding="utf-8")
