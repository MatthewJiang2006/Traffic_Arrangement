from dataclasses import dataclass
from math import pi

from logistics_park.common.messages import VehicleState
from .geometry import Projection, wrap_angle
from .road_graph import RoadGraph


@dataclass(frozen=True)
class Match:
    edge_id: str
    projection: Projection
    heading_error: float


def match_pose(graph: RoadGraph, x: float, y: float, heading: float,
               candidates: tuple[str, ...] | None = None) -> Match:
    """Use direction to distinguish coincident directed lanes."""
    candidates = tuple(graph.edges) if candidates is None else candidates
    if not candidates:
        raise ValueError("no matching candidates")
    matches = []
    for edge_id in candidates:
        projection = graph.edges[edge_id].centerline.project(x, y)
        error = wrap_angle(heading - projection.heading)
        score = projection.distance + 0.1 * abs(error) / pi
        matches.append((score, Match(edge_id, projection, error)))
    return min(matches, key=lambda item: item[0])[1]


def update_route_match(graph: RoadGraph, state: VehicleState, cyclic: bool = False) -> bool:
    """Match only current/next edge. Return True when advancing through route end."""
    index = state.route_index
    current = graph.edges[state.route[index]]
    projection = current.centerline.project(state.x, state.y)
    next_index = (index + 1) % len(state.route)
    wrapped = False
    if (cyclic or index + 1 < len(state.route)) and projection.s >= current.length - 0.12:
        following = graph.edges[state.route[next_index]]
        next_projection = following.centerline.project(state.x, state.y)
        if next_projection.distance + 1e-8 < projection.distance:
            wrapped = next_index == 0
            index, projection = next_index, next_projection
    state.route_index = index
    state.edge_id = state.route[index]
    state.s = projection.s
    state.lateral_offset = projection.lateral_offset
    state.heading_error = wrap_angle(state.heading - projection.heading)
    return wrapped
