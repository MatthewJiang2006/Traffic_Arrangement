from math import sqrt
import pytest

from logistics_park.common.enums import Maneuver, NodeType
from logistics_park.common.messages import Movement
from logistics_park.map.geometry import Polyline
from logistics_park.map.road_graph import Node,RoadEdge,RoadGraph
from logistics_park.routing.shortest_path import NoRouteError,shortest_path


@pytest.fixture
def restricted_graph():
    positions = {"s":(0,0),"a":(.5,0),"b":(0,1),"x":(1,0),"d":(2,0)}
    nodes = [Node(key,NodeType.WAYPOINT,value) for key,value in positions.items()]
    pairs = [("s","a"),("a","x"),("s","b"),("b","x"),("x","d")]
    edges = [RoadEdge(a+b,a,b,Polyline((positions[a],positions[b])),.2,1,2) for a,b in pairs]
    movements = [Movement(a+"_"+b,a,b,Maneuver.STRAIGHT,((0,0),(1,0)),(),1)
                 for a,b in [("sa","ax"),("sb","bx"),("bx","xd")]]
    return RoadGraph(nodes,edges,movements)


@pytest.mark.parametrize("algorithm",["dijkstra","astar"])
def test_search_retains_entering_edge_for_legal_turns(restricted_graph,algorithm):
    result = shortest_path(restricted_graph,"s","d",algorithm)
    assert result.edges == ("sb","bx","xd")
    assert result.travel_time == pytest.approx(2+sqrt(2))
    # The cheaper arrival through ax cannot turn into xd.
    with pytest.raises(NoRouteError):
        shortest_path(restricted_graph,"x","d",algorithm,incoming_edge="ax")
    assert shortest_path(restricted_graph,"x","d",algorithm,incoming_edge="bx").edges == ("xd",)


@pytest.mark.parametrize("algorithm",["dijkstra","astar"])
def test_directed_unreachable_blocked_and_empty_route(restricted_graph,algorithm):
    with pytest.raises(NoRouteError):
        shortest_path(restricted_graph,"d","s",algorithm)
    with pytest.raises(NoRouteError):
        shortest_path(restricted_graph,"s","d",algorithm,blocked_edges=frozenset({"bx"}))
    route = shortest_path(restricted_graph,"s","s",algorithm)
    assert route.edges == () and route.travel_time == 0


def test_routing_rejects_invalid_inputs(restricted_graph):
    for options in ({"origin":"unknown"},{"algorithm":"invalid"},{"incoming_edge":"xd"},
                    {"blocked_edges":frozenset({"unknown"})}):
        kwargs = {"origin":"s","destination":"d",**options}
        with pytest.raises(ValueError):
            shortest_path(restricted_graph,**kwargs)


def test_determinism_independent_of_graph_insertion_order(restricted_graph):
    graph = restricted_graph
    reverse = RoadGraph(list(reversed(list(graph.nodes.values()))),list(reversed(list(graph.edges.values()))),
                        list(reversed(list(graph.movements.values()))))
    assert shortest_path(graph,"s","d") == shortest_path(reverse,"s","d")


def test_astar_matches_dijkstra_for_every_park_endpoint(task_graph):
    for origin in task_graph.nodes:
        for destination in task_graph.nodes:
            dijkstra = shortest_path(task_graph,origin,destination)
            astar = shortest_path(task_graph,origin,destination,"astar")
            assert astar.travel_time == pytest.approx(dijkstra.travel_time)
            if astar.edges:
                task_graph.validate_route(astar.edges)
