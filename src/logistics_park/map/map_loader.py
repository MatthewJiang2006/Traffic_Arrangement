import json
from pathlib import Path

from logistics_park.common.enums import Maneuver, NodeType
from logistics_park.common.messages import Movement
from .geometry import Polyline
from .road_graph import Node, RoadEdge, RoadGraph


def load_map(path: str | Path) -> RoadGraph:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    nodes = [Node(item["node_id"], NodeType(item["node_type"]), tuple(item["position"]))
             for item in data["nodes"]]
    edges = [RoadEdge(item["edge_id"], item["from_node"], item["to_node"],
                      Polyline(tuple(tuple(point) for point in item["centerline"])),
                      item["width"], item["speed_limit"], item["storage_capacity"])
             for item in data["edges"]]
    movements = [Movement(item["movement_id"], item["incoming_edge"], item["outgoing_edge"],
                          Maneuver(item["maneuver"]), tuple(tuple(p) for p in item["reference_path"]),
                          tuple(item["conflict_zones"]), item["speed_limit"])
                 for item in data.get("movements", [])]
    return RoadGraph(nodes, edges, movements)
