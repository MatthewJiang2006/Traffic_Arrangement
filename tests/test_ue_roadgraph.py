"""Validation of the extracted, fixed UE road layout (no vehicle actuation)."""

import json
from pathlib import Path

from logistics_park.map.map_loader import load_map
from logistics_park.routing.shortest_path import shortest_path


ROOT = Path(__file__).resolve().parents[1]


def test_two_copies_form_one_routable_park():
    graph = load_map(ROOT / "config/topology/industrialarea_ue.json")
    survey = json.loads((ROOT / "config/topology/industrialarea_ue_survey.json").read_text())
    assert (len(graph.nodes), len(graph.edges), len(graph.movements)) == (100, 136, 172)
    assert survey["junction_count"] == 12
    assert len(survey["corridors"]) == 16
    seams = [link for link in survey["corridors"] if link["a"].startswith("j_1_") and link["b"].startswith("j_2_")]
    assert {(link["a"], link["b"]) for link in seams} == {("j_1_0", "j_2_0"), ("j_1_1", "j_2_1")}
    assert all(.55 < link["max_mesh_gap_m"] < .7 for link in seams)
    assert "road_j_1_2_j_2_2" not in graph.edges
    for origin, destination in (("gate_south_0_in", "gate_north_0_out"),
                                ("gate_south_1_in", "gate_north_1_out"),
                                ("gate_north_0_in_right", "gate_south_1_out")):
        routes = [shortest_path(graph, origin, destination, algorithm=name) for name in ("dijkstra", "astar")]
        assert routes[0].travel_time == routes[1].travel_time
        assert any(edge in routes[0].edges for edge in ("road_j_1_0_j_2_0", "road_j_1_1_j_2_1",
                                                       "road_j_2_0_j_1_0", "road_j_2_1_j_1_1"))
        graph.validate_route(routes[0].edges)


def test_road_arrows_restrict_only_signed_approaches():
    graph = load_map(ROOT / "config/topology/industrialarea_ue.json")
    survey = json.loads((ROOT / "config/topology/industrialarea_ue_survey.json").read_text())
    assert len(survey["arrow_restrictions"]) == 10
    assert len(survey["outward_arrows"]) == 0
    for rule in survey["arrow_restrictions"]:
        key, arm = rule["junction"], rule["incoming_arm"]
        straight = {"W": "E", "E": "W", "S": "N", "N": "S"}[arm]
        if f"{key}_out_{straight}" in graph.nodes:
            assert any(e.startswith(f"turn_{key}_{arm}_{straight}_") for e in graph.edges)


def test_two_lane_approaches_allow_straight_and_only_the_matching_turn():
    graph = json.loads((ROOT / "config/topology/industrialarea_ue.json").read_text())
    for edge in graph["edges"]:
        if edge["edge_id"].startswith("turn_"):
            allowed = {"LEFT": {"STRAIGHT", "LEFT"}, "RIGHT": {"STRAIGHT", "RIGHT"}, "SINGLE": {"STRAIGHT", "LEFT", "RIGHT"}}
            assert edge["maneuver"] in allowed[edge["incoming_lane"]]
            if edge["maneuver"] == "STRAIGHT":
                assert edge["incoming_lane"] == edge["outgoing_lane"]
    ids = {e["edge_id"] for e in graph["edges"]}
    for direction in (("j_1_0", "j_2_0"), ("j_2_0", "j_1_0")):
        base = "road_"+"_".join(direction)
        assert base in ids and base+"_right" in ids
    assert "road_j_1_1_j_2_1_right" not in ids
    assert "turn_j_2_0_S_E_left_single" in ids
    assert "turn_j_2_0_S_E_right_single" not in ids
    assert "turn_j_2_0_N_E_right_single" in ids
    assert "turn_j_2_0_N_E_left_single" not in ids


def test_connectors_match_ue_junction_bounds_and_downstream_edges():
    graph = load_map(ROOT / "config/topology/industrialarea_ue.json")
    controls = json.loads((ROOT / "config/traffic/industrialarea_ue_controls.json").read_text())
    assert len(controls["intersections"]) == 12
    for control in controls["intersections"]:
        x0, y0, x1, y1 = control["bounds"]
        assert x1 > x0 and y1 > y0
        for connector_id, downstream_id in control["connector_exits"].items():
            connector = graph.edges[connector_id]
            downstream = graph.edges[downstream_id]
            assert connector.to_node == downstream.from_node
            assert connector.storage_capacity == 1
            assert all(x0-.001 <= x <= x1+.001 and y0-.001 <= y <= y1+.001
                       for x, y in connector.centerline.points)


def test_original_spawn_snapshot_is_not_silently_treated_as_road_ready():
    spawns = json.loads((ROOT / "config/traffic/industrialarea_ue_spawns_initial.json").read_text())["spawns"]
    assert len(spawns) == 24
    assert sum(item["ready_for_roadgraph_spawn"] for item in spawns) == 10
    assert {item["vehicle_id"] for item in spawns if item["inside_junction_by_center"]} == {"Car_2_1", "Car_2_2"}
    assert all(not item["ready_for_roadgraph_spawn"] for item in spawns if item["vehicle_id"].startswith(("Car_4_", "Car_5_", "Car_6_", "Car_7_")))


def test_calibrated_vehicles_are_on_painted_lanes_with_settled_spawn_height():
    survey = json.loads((ROOT / "config/topology/industrialarea_ue_survey.json").read_text())
    plan = json.loads((ROOT / "config/traffic/industrialarea_ue_spawn_plan.json").read_text())
    spawns = json.loads((ROOT / "config/traffic/industrialarea_ue_spawns.json").read_text())["spawns"]
    assert len(plan["targets"]) == 24
    assert sum(item["ready_for_roadgraph_spawn"] for item in spawns) == 24
    assert 1.8 < survey["lane_offset_m"] < 2.0
    assert plan["minimum_body_clearance_m"] >= 2.0
    for item in plan["targets"]:
        x, y, z = item["target_ue_position_m"]
        row = int(item["road_edge"].split("_")[2])
        road_y = survey["junctions"][f"j_{row}_0"]["center_m"][1]
        expected_y = road_y + (survey["lane_offset_m"] if item["target_ue_yaw_deg"] == 0 else -survey["lane_offset_m"])
        assert abs(y-expected_y) < .001
        assert .86 < z < .92


def test_ue_road_lanes_follow_vehicle_right_axis_in_all_four_directions():
    graph = load_map(ROOT / "config/topology/industrialarea_ue.json")
    survey = json.loads((ROOT / "config/topology/industrialarea_ue_survey.json").read_text())
    for edge_id, junction, right in (
        ("road_j_0_0_j_0_1", "j_0_0", (0, 1)),
        ("road_j_0_1_j_0_0", "j_0_1", (0, -1)),
        ("road_j_0_0_j_1_0", "j_0_0", (-1, 0)),
        ("road_j_1_0_j_0_0", "j_1_0", (1, 0)),
    ):
        point = graph.edges[edge_id].centerline.points[0]
        centre = survey["junctions"][junction]["center_m"]
        right_offset = sum((point[i]-centre[i])*right[i] for i in (0, 1))
        assert abs(right_offset-survey["lane_offset_m"]) < .001


def test_ue_turn_classification_matches_body_axes():
    import runpy
    classify = runpy.run_path(str(ROOT / "scripts/build_ue_roadgraph.py"))["maneuver"]
    assert classify((1, 0), (0, 1)) == "RIGHT"
    assert classify((1, 0), (0, -1)) == "LEFT"
    assert classify((0, 1), (-1, 0)) == "RIGHT"
    assert classify((0, 1), (1, 0)) == "LEFT"
