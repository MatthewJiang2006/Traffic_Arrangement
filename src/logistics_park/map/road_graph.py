from dataclasses import dataclass
from math import hypot

from logistics_park.common.enums import NodeType
from logistics_park.common.messages import Movement, Point, finite
from .geometry import Polyline


@dataclass(frozen=True)
class Node:
    node_id: str
    node_type: NodeType
    position: Point

    def __post_init__(self) -> None:
        finite(*self.position)
        if not self.node_id:
            raise ValueError("empty node ID")


@dataclass(frozen=True)
class RoadEdge:
    edge_id: str
    from_node: str
    to_node: str
    centerline: Polyline
    width: float
    speed_limit: float
    storage_capacity: int

    def __post_init__(self) -> None:
        finite(self.width, self.speed_limit)
        if not all((self.edge_id, self.from_node, self.to_node)) or min(self.width, self.speed_limit) <= 0 or self.storage_capacity < 0:
            raise ValueError("invalid edge")

    @property
    def length(self) -> float:
        return self.centerline.length


class RoadGraph:
    def __init__(self, nodes: list[Node], edges: list[RoadEdge], movements: list[Movement] | None = None):
        self.nodes = {node.node_id: node for node in nodes}
        self.edges = {edge.edge_id: edge for edge in edges}
        movements = movements or []
        self.movements = {movement.movement_id: movement for movement in movements}
        if not nodes or not edges:
            raise ValueError("map cannot be empty")
        if len(self.nodes) != len(nodes) or len(self.edges) != len(edges) or len(self.movements) != len(movements):
            raise ValueError("duplicate map IDs")
        self.outgoing: dict[str, list[str]] = {node_id: [] for node_id in self.nodes}
        self.incoming: dict[str, list[str]] = {node_id: [] for node_id in self.nodes}
        for edge in edges:
            if edge.from_node not in self.nodes or edge.to_node not in self.nodes:
                raise ValueError(f"unknown endpoint on {edge.edge_id}")
            for point, node_id in ((edge.centerline.points[0], edge.from_node),
                                   (edge.centerline.points[-1], edge.to_node)):
                node = self.nodes[node_id]
                if hypot(point[0]-node.position[0], point[1]-node.position[1]) > 1e-6:
                    raise ValueError(f"centerline endpoint mismatch on {edge.edge_id}")
            self.outgoing[edge.from_node].append(edge.edge_id)
            self.incoming[edge.to_node].append(edge.edge_id)
        for movement in movements:
            if not self.is_connected(movement.incoming_edge, movement.outgoing_edge):
                raise ValueError(f"disconnected movement {movement.movement_id}")

    def is_connected(self, incoming: str, outgoing: str) -> bool:
        return (incoming in self.edges and outgoing in self.edges
                and self.edges[incoming].to_node == self.edges[outgoing].from_node)

    def is_legal_turn(self, incoming: str, outgoing: str) -> bool:
        """Turns require explicit movements; topology alone does not authorize them."""
        return any(m.incoming_edge == incoming and m.outgoing_edge == outgoing
                   for m in self.movements.values())

    def validate_route(self, route: tuple[str, ...], cyclic: bool = False) -> None:
        if not route or any(edge not in self.edges for edge in route):
            raise ValueError("route contains unknown edges")
        pairs = list(zip(route, route[1:]))
        if cyclic:
            pairs.append((route[-1], route[0]))
        if any(not self.is_legal_turn(a, b) for a, b in pairs):
            raise ValueError("route contains an illegal turn")
