from dataclasses import replace
from math import pi
import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.map.geometry import Polyline
from logistics_park.map.map_matching import match_pose, update_route_match
from logistics_park.map.road_graph import Node, RoadEdge, RoadGraph
from logistics_park.common.enums import NodeType


def test_polyline_projection_and_interpolation():
    line = Polyline(((0,0),(1,0),(1,1)))
    assert line.length == 2
    projection = line.project(1.2,0.4)
    assert projection.s == pytest.approx(1.4)
    assert projection.lateral_offset == pytest.approx(-0.2)
    assert line.at(1.5)[0] == (1,0.5)
    assert line.at(100)[0] == (1,1)
    with pytest.raises(ValueError):
        Polyline(((0,0),(0,0)))


def test_topology_and_turn_validation(graph):
    graph.validate_route(tuple(graph.edges), cyclic=True)
    assert graph.is_legal_turn("e0", "e1")
    assert not graph.is_legal_turn("e0", "e2")
    with pytest.raises(ValueError):
        graph.validate_route(("e0", "e2"))
    with pytest.raises(ValueError):
        RoadGraph(list(graph.nodes.values()), list(graph.edges.values())+[graph.edges["e0"]])
    with pytest.raises(ValueError):
        RoadGraph(list(graph.nodes.values()), [replace(graph.edges["e0"], to_node="missing")])
    with pytest.raises(ValueError):
        RoadGraph(list(graph.nodes.values()), [replace(graph.edges["e0"], centerline=Polyline(((0,0),(1,1))))])


def test_direction_disambiguates_reverse_lane():
    nodes = [Node("a", NodeType.GATE, (0,0)),Node("b",NodeType.GATE,(1,0))]
    edges = [RoadEdge("ab","a","b",Polyline(((0,0),(1,0))),0.2,0.3,3),
             RoadEdge("ba","b","a",Polyline(((1,0),(0,0))),0.2,0.3,3)]
    graph = RoadGraph(nodes,edges)
    assert match_pose(graph,0.4,0.01,0).edge_id == "ab"
    assert match_pose(graph,0.4,0.01,pi).edge_id == "ba"


def test_route_boundary_and_wrap(graph, config):
    state = VehicleState("v", 1.58,0.203,0, route=config.route, edge_id="e0", s=1.1)
    assert not update_route_match(graph,state,cyclic=True)
    assert state.edge_id == "e1" and 0 < state.s < 0.1
    state.route_index, state.x, state.y = 7, 0.47, 0.2
    assert update_route_match(graph,state,cyclic=True)
    assert state.edge_id == "e0" and state.s == pytest.approx(0.02)
