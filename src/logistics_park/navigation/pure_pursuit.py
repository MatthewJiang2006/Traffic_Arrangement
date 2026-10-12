from dataclasses import dataclass
from math import cos, hypot, sin, sqrt

from logistics_park.common.messages import ControlCommand, VehicleState, finite
from logistics_park.map.road_graph import RoadGraph


@dataclass(frozen=True)
class TrackingConfig:
    cruise_speed: float = 0.25
    lookahead_base: float = 0.08
    lookahead_gain: float = 0.3
    lateral_gain: float = 0.5
    heading_gain: float = 0.3
    lateral_acceleration: float = 0.3

    def __post_init__(self) -> None:
        finite(*self.__dict__.values())
        if min(self.cruise_speed, self.lookahead_base, self.lateral_acceleration) <= 0:
            raise ValueError("positive speed, lookahead and acceleration required")
        if min(self.lookahead_gain, self.lateral_gain, self.heading_gain) < 0:
            raise ValueError("gains cannot be negative")


def point_ahead(graph: RoadGraph, state: VehicleState, distance: float, cyclic: bool):
    remaining = distance + state.s
    index = state.route_index
    for _ in range(len(state.route) + 1):
        edge = graph.edges[state.route[index]]
        if remaining <= edge.length or (not cyclic and index == len(state.route)-1):
            return edge.centerline.at(min(remaining, edge.length))[0]
        remaining -= edge.length
        index = (index+1) % len(state.route)
    raise ValueError("lookahead exceeds route length")


class PurePursuit:
    def __init__(self, config: TrackingConfig):
        self.config = config

    def command(self, graph: RoadGraph, state: VehicleState, sequence: int,
                speed_cap: float = float("inf"), cyclic: bool = False) -> ControlCommand:
        cfg = self.config
        lookahead = cfg.lookahead_base + cfg.lookahead_gain*state.speed
        target = point_ahead(graph, state, lookahead, cyclic)
        dx, dy = target[0]-state.x, target[1]-state.y
        distance_sq = dx*dx + dy*dy
        lateral = -sin(state.heading)*dx + cos(state.heading)*dy
        curvature = 2*lateral/max(distance_sq, 1e-8)
        speed = min(cfg.cruise_speed, graph.edges[state.edge_id].speed_limit, speed_cap,
                    sqrt(cfg.lateral_acceleration/max(abs(curvature), 1e-6)))
        speed = max(0.0, speed)
        if not cyclic and state.route_index == len(state.route)-1:
            remaining = graph.edges[state.edge_id].length-state.s
            speed = min(speed, max(0.0, 1.5*remaining))
            if hypot(dx, dy) < 0.01:
                speed = 0.0
        omega = (speed*curvature - cfg.lateral_gain*state.lateral_offset
                 - cfg.heading_gain*state.heading_error)
        return ControlCommand(state.vehicle_id, sequence, state.timestamp, speed, omega)
