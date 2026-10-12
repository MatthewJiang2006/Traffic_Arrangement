"""Static shortest travel time with explicit movement restrictions."""
from dataclasses import dataclass
from heapq import heappop, heappush
from math import hypot

from logistics_park.map.road_graph import RoadGraph


class NoRouteError(ValueError):
    pass


@dataclass(frozen=True)
class Route:
    origin: str
    destination: str
    edges: tuple[str, ...]
    travel_time: float


def shortest_path(graph: RoadGraph, origin: str, destination: str,
                  algorithm: str = "dijkstra", incoming_edge: str | None = None,
                  blocked_edges: frozenset[str] = frozenset()) -> Route:
    if algorithm not in ("dijkstra", "astar"):
        raise ValueError("algorithm must be dijkstra or astar")
    if origin not in graph.nodes or destination not in graph.nodes:
        raise ValueError("unknown routing endpoint")
    if incoming_edge is not None and (incoming_edge not in graph.edges or graph.edges[incoming_edge].to_node != origin):
        raise ValueError("incoming edge does not end at origin")
    if any(edge not in graph.edges for edge in blocked_edges):
        raise ValueError("unknown blocked edge")
    max_speed = max(edge.speed_limit for edge in graph.edges.values())
    target = graph.nodes[destination].position

    def heuristic(node_id: str) -> float:
        if algorithm == "dijkstra":
            return 0.0
        x, y = graph.nodes[node_id].position
        return hypot(target[0]-x, target[1]-y)/max_speed

    # Entering edge is part of the state: reaching the same node through a
    # cheaper edge must not discard a more costly but legal outgoing movement.
    start = (origin, incoming_edge or "")
    best = {start: (0.0, ())}
    queue = [(heuristic(origin), 0.0, (), *start)]
    while queue:
        _, cost, path, node_id, previous = heappop(queue)
        if best.get((node_id, previous)) != (cost, path):
            continue
        if node_id == destination:
            return Route(origin, destination, path, cost)
        for edge_id in sorted(graph.outgoing[node_id]):
            if edge_id in blocked_edges or (previous and not graph.is_legal_turn(previous, edge_id)):
                continue
            edge = graph.edges[edge_id]
            candidate = cost + edge.length/edge.speed_limit
            next_path = path + (edge_id,)
            state = (edge.to_node, edge_id)
            if state not in best or (candidate, next_path) < best[state]:
                best[state] = (candidate, next_path)
                heappush(queue, (candidate+heuristic(edge.to_node), candidate, next_path, *state))
    raise NoRouteError(f"no legal route from {origin} to {destination}")
