#!/usr/bin/env python3
"""Build the IndustrialArea RoadGraph from a read-only UE actor snapshot.

This captures the actual single-lane roads and the two-lane west road. It does not certify
truck turning clearance or select warehouse/service locations.
"""

from __future__ import annotations

import argparse
import json
from html import escape
from math import atan2, cos, floor, hypot, pi, radians, sin
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
import sys
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.turn_geometry import right_turn_path

ROAD_MESHES = {"SM_Road_3", "SM_Road_4", "SM_Road_6", "SM_Road_8", "SM_Road_9", "SM_Road_10"}
JUNCTION_MESHES = {"SM_Road_3", "SM_Road_4", "SM_Road_6"}
LANE_OFFSET_M = 1.9  # Measured from the centres of the straight/left road-arrow decals.
ROAD_SPEED_M_S = 5.0  # Conservative routing placeholder; calibrate against the truck.
TURN_SPEED_M_S = 2.0
TRUCK_LENGTH_M = 8.35  # AirSim pawn bounding box is approximately 8.34 x 3.84 m.
STANDSTILL_GAP_M = 2.0


def mesh_name(actor: dict) -> str | None:
    for path in actor["meshes"]:
        name = path.rsplit(".", 1)[-1]
        if name in ROAD_MESHES:
            return name
    return None


def center(actor: dict) -> tuple[float, float]:
    return tuple(value / 100 for value in actor["bounds_origin_cm"][:2])


def extents(actor: dict) -> tuple[float, float]:
    return tuple(value / 100 for value in actor["bounds_extent_cm"][:2])


def rounded(point: tuple[float, float]) -> list[float]:
    return [round(point[0], 4), round(point[1], 4)]


def bezier(start, end, start_heading, end_heading, handle):
    controls = [start, (start[0] + start_heading[0]*handle, start[1] + start_heading[1]*handle),
                (end[0] - end_heading[0]*handle, end[1] - end_heading[1]*handle), end]
    points = []
    for i in range(13):
        t = i/12
        x = sum(weight*point[0] for weight, point in zip(((1-t)**3, 3*(1-t)**2*t, 3*(1-t)*t*t, t**3), controls))
        y = sum(weight*point[1] for weight, point in zip(((1-t)**3, 3*(1-t)**2*t, 3*(1-t)*t*t, t**3), controls))
        points.append(rounded((x, y)))
    return points


def maneuver(incoming, outgoing):
    cross = incoming[0]*outgoing[1] - incoming[1]*outgoing[0]
    # UE: +X forward, +Y right. Positive yaw is a right turn.
    return "STRAIGHT" if abs(cross) < 0.5 else "RIGHT" if cross > 0 else "LEFT"


def expand_lanes(graph, controls, junctions, restricted):
    """Represent the two marked lanes separately, including approach legality."""
    spacing = 2*LANE_OFFSET_M
    nodes = {n["node_id"]: dict(n, lane_side="SINGLE") for n in graph["nodes"]}
    roads = [e for e in graph["edges"] if e["edge_id"].startswith("road_")]
    incoming, outgoing = {}, {}
    lane_roads = []
    for road in roads:
        a, b = road["centerline"][0], road["centerline"][-1]
        length = hypot(b[0]-a[0], b[1]-a[1])
        forward = ((b[0]-a[0])/length, (b[1]-a[1])/length)
        right = (-forward[1], forward[0])
        wide = abs(forward[1]) > .9 and abs((a[0]+b[0])/2-junctions["j_0_0"]["center"][0]) < 7
        for side in (("LEFT", "RIGHT") if wide else ("SINGLE",)):
            suffix = "_right" if side == "RIGHT" else ""
            path = road["centerline"] if side != "RIGHT" else [
                rounded((p[0]+spacing*right[0], p[1]+spacing*right[1])) for p in road["centerline"]]
            for base_id, point in ((road["from_node"], path[0]), (road["to_node"], path[-1])):
                nodes[base_id+suffix] = dict(nodes[base_id], node_id=base_id+suffix, position=point, lane_side=side)
            item = dict(road, edge_id=road["edge_id"]+suffix, from_node=road["from_node"]+suffix,
                        to_node=road["to_node"]+suffix, centerline=path, width=spacing, lane_side=side)
            lane_roads.append(item)
            if "_in_" in road["to_node"]:
                key, arm = road["to_node"].split("_in_")
                incoming[key, arm, side] = (item, forward)
            if "_out_" in road["from_node"]:
                key, arm = road["from_node"].split("_out_")
                outgoing[key, arm, side] = (item, forward)
    connectors = []
    new_controls = []
    for control in controls:
        key = control["intersection_id"]
        exits = {}
        for (j, arm, side), (before, heading) in incoming.items():
            if j != key:
                continue
            for (k, end_arm, end_side), (after, end_heading) in outgoing.items():
                if k != key or arm == end_arm:
                    continue
                kind = maneuver(heading, end_heading)
                if side == "SINGLE" and (key, arm) in restricted and kind == "RIGHT":
                    continue
                if kind == "LEFT" and side == "RIGHT" or kind == "RIGHT" and side == "LEFT":
                    continue
                if kind == "STRAIGHT" and side != end_side:
                    continue
                start, end = before["centerline"][-1], after["centerline"][0]
                # Avoid an oversized Bezier handle looping out of a tight
                # outer-lane corner. Turns may enter either outgoing lane;
                # straight traffic keeps its lane.
                handle = min(junctions[key]["half"])*.55
                if kind != "STRAIGHT":
                    handle = min(handle, min(abs(end[0]-start[0]), abs(end[1]-start[1]))*.55)
                path = bezier(start, end, heading, end_heading, handle)
                if kind=='RIGHT':
                    path=right_turn_path(dict(centerline=path))
                connector_id = f"turn_{key}_{arm}_{end_arm}_{side.lower()}_{end_side.lower()}"
                length = sum(hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:]))
                connectors.append({"edge_id": connector_id, "from_node": before["to_node"],
                    "to_node": after["from_node"], "centerline": path, "width": spacing,
                    "speed_limit": min(TURN_SPEED_M_S,1.5) if kind=='RIGHT' else TURN_SPEED_M_S, "storage_capacity": 1,
                    "incoming_lane": side, "outgoing_lane": end_side, "maneuver": kind})
                if kind=='RIGHT':
                    connectors[-1].update(maximum_path_error_m=1.2,right_turn_policy='delayed_yaw_lane_dependent_radius',
                        straight_until_m=hypot(path[1][0]-path[0][0],path[1][1]-path[0][1]))
                exits[connector_id] = after["edge_id"]
        new_controls.append(dict(control, connector_exits=exits))
    edges = lane_roads+connectors
    by_from, by_to = {}, {}
    for e in edges:
        by_from.setdefault(e["from_node"], []).append(e)
        by_to.setdefault(e["to_node"], []).append(e)
    movements = []
    for key in nodes:
        for before in by_to.get(key, []):
            for after in by_from.get(key, []):
                movements.append({"movement_id": before["edge_id"]+"__"+after["edge_id"],
                    "incoming_edge": before["edge_id"], "outgoing_edge": after["edge_id"],
                    "maneuver": before.get("maneuver", after.get("maneuver", "STRAIGHT")),
                    "reference_path": [before["centerline"][-2], before["centerline"][-1], after["centerline"][1]],
                    "conflict_zones": [], "speed_limit": min(before["speed_limit"], after["speed_limit"])})
    return dict(graph, nodes=list(nodes.values()), edges=edges, movements=movements,
                description="Unified UE park, right-hand traffic; west road two lanes per direction, other roads one"), new_controls


def make_graph(snapshot: dict) -> tuple[dict, dict, dict]:
    if "IndustrialArea（复件）" not in snapshot["map_path"]:
        raise ValueError("unexpected UE map")
    roads = [actor for actor in snapshot["actors"] if actor["class"] == "StaticMeshActor" and mesh_name(actor)]
    intersections = [actor for actor in roads if mesh_name(actor) in JUNCTION_MESHES]
    if len(roads) != 86 or len(intersections) != 12:
        raise ValueError("UE road asset count changed; inspect scene before regenerating")
    xs = sorted({round(center(actor)[0], 1) for actor in intersections})
    ys = sorted({round(center(actor)[1], 1) for actor in intersections})
    if len(xs) != 3 or len(ys) != 4:
        raise ValueError("expected three road columns and four junction rows")

    def junction_id(column: int, row: int) -> str:
        return f"j_{row}_{column}"

    junctions = {}
    for actor in intersections:
        x, y = center(actor)
        col = min(range(3), key=lambda index: abs(xs[index]-x))
        row = min(range(4), key=lambda index: abs(ys[index]-y))
        key = junction_id(col, row)
        if key in junctions or abs(xs[col]-x) > .2 or abs(ys[row]-y) > .2:
            raise ValueError("ambiguous intersection geometry")
        dx, dy = extents(actor)
        junctions[key] = {"center": (x, y), "half": (dx, dy), "actor": actor["path"],
                          "region": "lower" if row < 2 else "upper"}

    # Adjacent corridors are included only when matching road tiles cover them.
    links = []

    def add_link(a: str, b: str, orientation: str):
        first, second = junctions[a], junctions[b]
        if orientation == "horizontal":
            lo = first["center"][0] + first["half"][0]
            hi = second["center"][0] - second["half"][0]
            coordinate = (first["center"][1] + second["center"][1])/2
            tiles = [t for t in roads if mesh_name(t) == "SM_Road_8"
                     and abs(center(t)[1]-coordinate) < .3]
            intervals = sorted((center(t)[0]-extents(t)[0], center(t)[0]+extents(t)[0]) for t in tiles)
        else:
            lo = first["center"][1] + first["half"][1]
            hi = second["center"][1] - second["half"][1]
            coordinate = (first["center"][0] + second["center"][0])/2
            allowed_mesh = "SM_Road_10" if abs(coordinate-xs[0]) < 1 else "SM_Road_9"
            tiles = [t for t in roads if mesh_name(t) == allowed_mesh
                     and abs(center(t)[0]-coordinate) < .3]
            intervals = sorted((center(t)[1]-extents(t)[1], center(t)[1]+extents(t)[1]) for t in tiles)
        cursor = lo
        max_gap = 0.0
        for start, end in intervals:
            if end < lo or start > hi:
                continue
            max_gap = max(max_gap, max(0.0, start-cursor))
            cursor = max(cursor, end)
        max_gap = max(max_gap, max(0.0, hi-cursor))
        if max_gap > .75:
            raise ValueError(f"road gap {max_gap:.2f} m between {a} and {b}")
        links.append({"a": a, "b": b, "orientation": orientation, "max_mesh_gap_m": round(max_gap, 3)})

    for row in range(4):
        for col in range(2):
            add_link(junction_id(col, row), junction_id(col+1, row), "horizontal")
    for col in range(3):
        for row in range(3):
            if col == 2 and row == 1:
                continue  # No road mesh joins the two copies on the right.
            add_link(junction_id(col, row), junction_id(col, row+1), "vertical")
    if len(links) != 16:
        raise AssertionError("unexpected road connectivity")

    # Read the straight-and-left arrows from actual decal transforms. UE decal
    # local -right points in the arrow's direction for this material, verified
    # in a top view with UE +X upwards and +Y to the screen's right.
    arrow_rules = []
    outward_arrows = []
    for actor in snapshot["actors"]:
        if not any("/Mat_RoadMarking_1." in (item["material"] or "") for item in actor["decals"]):
            continue
        x, y = (v/100 for v in actor["location_cm"][:2])
        rx, ry = (-v for v in actor["decal_right_vector"][:2])
        if abs(rx) > abs(ry):
            direction = "W" if rx < 0 else "E"
            candidates = [(key, j) for key, j in junctions.items()
                          if abs(j["center"][1]-y) < 4 and (j["center"][0]-x)*rx > 0]
            arm = "E" if direction == "W" else "W"
        else:
            direction = "S" if ry < 0 else "N"
            candidates = [(key, j) for key, j in junctions.items()
                          if abs(j["center"][0]-x) < 4 and (j["center"][1]-y)*ry > 0]
            arm = "N" if direction == "S" else "S"
        if not candidates:
            outward_arrows.append(actor["path"])
            continue
        key, junction = min(candidates, key=lambda item: hypot(item[1]["center"][0]-x, item[1]["center"][1]-y))
        arrow_rules.append({"actor": actor["path"], "junction": key, "incoming_arm": arm,
                            "allowed": ["STRAIGHT", "LEFT"], "heading": direction})
    if len(arrow_rules) + len(outward_arrows) != 10:
        raise ValueError("road arrow count changed")
    restricted = {(rule["junction"], rule["incoming_arm"]) for rule in arrow_rules}
    if len(restricted) != len(arrow_rules):
        raise ValueError("duplicate arrow restriction assignment")

    nodes = []
    edges = []
    positions = {}
    arms = {key: set() for key in junctions}

    def node(key, point, kind="INTERSECTION"):
        if key in positions:
            raise ValueError(f"duplicate node {key}")
        point = rounded(point)
        positions[key] = point
        nodes.append({"node_id": key, "node_type": kind, "position": point})

    def edge(key, start, end, path=None, speed=ROAD_SPEED_M_S, capacity=None):
        path = path or [positions[start], positions[end]]
        length = sum(hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(path, path[1:]))
        if length <= .01:
            raise ValueError(f"degenerate road edge {key}")
        edges.append({"edge_id": key, "from_node": start, "to_node": end,
                      "centerline": path, "width": 5.5, "speed_limit": speed,
                      "storage_capacity": capacity if capacity is not None else
                      max(1, floor((length+STANDSTILL_GAP_M)/(TRUCK_LENGTH_M+STANDSTILL_GAP_M)))})

    for link in links:
        a, b = link["a"], link["b"]
        if link["orientation"] == "horizontal":
            arms[a].add("E"); arms[b].add("W")
        else:
            arms[a].add("N"); arms[b].add("S")
    for col in (0, 1):
        arms[junction_id(col, 0)].add("S")
        arms[junction_id(col, 3)].add("N")

    incoming_heading = {"W": (1, 0), "E": (-1, 0), "S": (0, 1), "N": (0, -1)}
    outgoing_heading = {"W": (-1, 0), "E": (1, 0), "S": (0, -1), "N": (0, 1)}
    for key, j in junctions.items():
        x, y = j["center"]
        hx, hy = j["half"]
        for arm in sorted(arms[key]):
            if arm == "W":
                enter, leave = (x-hx, y+LANE_OFFSET_M), (x-hx, y-LANE_OFFSET_M)
            elif arm == "E":
                enter, leave = (x+hx, y-LANE_OFFSET_M), (x+hx, y+LANE_OFFSET_M)
            elif arm == "S":
                enter, leave = (x-LANE_OFFSET_M, y-hy), (x+LANE_OFFSET_M, y-hy)
            else:
                enter, leave = (x+LANE_OFFSET_M, y+hy), (x-LANE_OFFSET_M, y+hy)
            node(f"{key}_in_{arm}", enter)
            node(f"{key}_out_{arm}", leave)

    connector_exits = {key: {} for key in junctions}
    for link in links:
        a, b = link["a"], link["b"]
        if link["orientation"] == "horizontal":
            edge(f"road_{a}_{b}", f"{a}_out_E", f"{b}_in_W")
            edge(f"road_{b}_{a}", f"{b}_out_W", f"{a}_in_E")
        else:
            edge(f"road_{a}_{b}", f"{a}_out_N", f"{b}_in_S")
            edge(f"road_{b}_{a}", f"{b}_out_S", f"{a}_in_N")

    terminal_y = {"south": min(center(t)[1]-extents(t)[1] for t in roads if mesh_name(t) in ("SM_Road_9", "SM_Road_10")),
                  "north": max(center(t)[1]+extents(t)[1] for t in roads if mesh_name(t) in ("SM_Road_9", "SM_Road_10"))}
    for col in (0, 1):
        south = junction_id(col, 0)
        north = junction_id(col, 3)
        for side, j_id in (("south", south), ("north", north)):
            x = junctions[j_id]["center"][0]
            y = terminal_y[side]
            incoming = f"gate_{side}_{col}_in"
            outgoing = f"gate_{side}_{col}_out"
            if side == "south":
                node(incoming, (x-LANE_OFFSET_M, y), "GATE")
                node(outgoing, (x+LANE_OFFSET_M, y), "GATE")
                edge(f"road_{incoming}_{j_id}", incoming, f"{j_id}_in_S")
                edge(f"road_{j_id}_{outgoing}", f"{j_id}_out_S", outgoing)
            else:
                node(incoming, (x+LANE_OFFSET_M, y), "GATE")
                node(outgoing, (x-LANE_OFFSET_M, y), "GATE")
                edge(f"road_{incoming}_{j_id}", incoming, f"{j_id}_in_N")
                edge(f"road_{j_id}_{outgoing}", f"{j_id}_out_N", outgoing)

    for key, j in sorted(junctions.items()):
        for incoming in sorted(arms[key]):
            for outgoing in sorted(arms[key]):
                if outgoing == incoming:
                    continue
                kind = maneuver(incoming_heading[incoming], outgoing_heading[outgoing])
                if (key, incoming) in restricted and kind == "RIGHT":
                    continue
                start = f"{key}_in_{incoming}"
                end = f"{key}_out_{outgoing}"
                connector = f"turn_{key}_{incoming}_{outgoing}"
                curve = bezier(positions[start], positions[end], incoming_heading[incoming],
                               outgoing_heading[outgoing], min(j["half"])*.55)
                edge(connector, start, end, curve, TURN_SPEED_M_S, 1)
                connector_exits[key][connector] = None

    incoming_by_node = {}
    outgoing_by_node = {}
    for item in edges:
        incoming_by_node.setdefault(item["to_node"], []).append(item)
        outgoing_by_node.setdefault(item["from_node"], []).append(item)
    movements = []
    for node_id in sorted(positions):
        for a in incoming_by_node.get(node_id, []):
            for b in outgoing_by_node.get(node_id, []):
                p0, p1, p2 = a["centerline"][-2], a["centerline"][-1], b["centerline"][1]
                before = atan2(p1[1]-p0[1], p1[0]-p0[0])
                after = atan2(p2[1]-p1[1], p2[0]-p1[0])
                angle = (after-before+pi) % (2*pi)-pi
                movements.append({"movement_id": a["edge_id"]+"__"+b["edge_id"],
                                  "incoming_edge": a["edge_id"], "outgoing_edge": b["edge_id"],
                                  "maneuver": "STRAIGHT" if abs(angle) < .25 else "RIGHT" if angle > 0 else "LEFT",
                                  "reference_path": [p0, p1, p2], "conflict_zones": [],
                                  "speed_limit": min(a["speed_limit"], b["speed_limit"])})
    outgoing_road = {item["from_node"]: item["edge_id"] for item in edges if item["edge_id"].startswith("road_")}
    controls = []
    for key, j in sorted(junctions.items()):
        exits = {}
        for connector in connector_exits[key]:
            end = next(item["to_node"] for item in edges if item["edge_id"] == connector)
            exits[connector] = outgoing_road[end]
        x, y = j["center"]
        hx, hy = j["half"]
        controls.append({"intersection_id": key, "bounds": rounded((x-hx, y-hy))+rounded((x+hx, y+hy)),
                         "connector_exits": exits})

    graph = {"description": "IndustrialArea（复件） and IndustrialArea_Copy unified UE road layout; UE world XY metres",
             "nodes": nodes, "edges": edges, "movements": movements}
    graph, controls = expand_lanes(graph, controls, junctions, restricted)
    edges = graph["edges"]
    def distance_to_segment(point, a, b):
        dx, dy = b[0]-a[0], b[1]-a[1]
        t = max(0.0, min(1.0, ((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)))
        return hypot(point[0]-a[0]-t*dx, point[1]-a[1]-t*dy)

    spawns = []
    for vehicle in snapshot["pie_vehicles"]:
        ue_position = tuple((snapshot["world_origin_cm"][i] + vehicle["location_from_origin_cm"][i])/100
                            for i in range(3))
        yaw = radians(vehicle["rotation_deg"][1])
        heading = (cos(yaw), sin(yaw))
        road_distances = [(min(distance_to_segment(ue_position, a, b)
                               for a, b in zip(item["centerline"], item["centerline"][1:])), item["edge_id"])
                          for item in edges if item["edge_id"].startswith("road_")
                          and sum(heading[i]*(item["centerline"][-1][i]-item["centerline"][0][i]) for i in (0, 1))
                          /hypot(item["centerline"][-1][0]-item["centerline"][0][0],
                                 item["centerline"][-1][1]-item["centerline"][0][1]) > .8]
        distance, nearest = min(road_distances)
        inside_junction = [control["intersection_id"] for control in controls
                           if control["bounds"][0] <= ue_position[0] <= control["bounds"][2]
                           and control["bounds"][1] <= ue_position[1] <= control["bounds"][3]]
        spawns.append({"vehicle_id": vehicle["name"], "ue_position_m": [round(v, 4) for v in ue_position],
                       "ue_yaw_deg": round(vehicle["rotation_deg"][1], 3),
                       "pawn_half_extent_m": [round(v/100, 3) for v in vehicle["bounds_extent_cm"]],
                       "nearest_aligned_road_edge": nearest, "distance_to_aligned_lane_m": round(distance, 3),
                       "inside_junction_by_center": inside_junction,
                       "ready_for_roadgraph_spawn": distance <= 2.75 and not inside_junction})
    survey = {"map_path": snapshot["map_path"], "world_origin_cm_at_capture": snapshot["world_origin_cm"],
              "coordinate_frame": "UE absolute world XY, metres; adapter must subtract runtime origin and invert UE Z for AirSim NED",
              "road_tile_count": len(roads), "junction_count": len(junctions),
              "junctions": {key: {"center_m": rounded(j["center"]), "half_extent_m": rounded(j["half"]),
                                  "region": j["region"], "source_actor": j["actor"]} for key, j in sorted(junctions.items())},
              "corridors": links, "arrow_restrictions": arrow_rules, "outward_arrows": outward_arrows,
              "lane_offset_m": LANE_OFFSET_M, "modelled_lanes_per_direction": {"west_vertical": 2, "other_roads": 1},
              "lane_offsets_m": {"LEFT": LANE_OFFSET_M, "RIGHT": 3*LANE_OFFSET_M},
              "lane_rules": {"LEFT": ["STRAIGHT", "LEFT"], "RIGHT": ["STRAIGHT", "RIGHT"]},
              "driving_side": "right; UE right vector=(-sin(yaw), cos(yaw))",
              "road_arrow_forward": "negative decal_right_vector",
              "road_speed_m_s_provisional": ROAD_SPEED_M_S, "turn_speed_m_s_provisional": TURN_SPEED_M_S,
              "capacity_vehicle_length_m": TRUCK_LENGTH_M, "capacity_standstill_gap_m": STANDSTILL_GAP_M,
              "pie_vehicle_count_at_capture": len(snapshot["pie_vehicles"]),
              "unverified": ["truck turn clearance", "UE surface collision across the two 0.6 m seams",
                             "speed limits and braking",
                             "warehouse/service/gate task semantics"]}
    return graph, {"intersections": controls}, survey, {"coordinate_frame": survey["coordinate_frame"],
                                                        "spawns": spawns}


def write_svg(snapshot: dict, graph: dict, survey: dict, spawns: dict, path: Path) -> None:
    roads = [actor for actor in snapshot["actors"] if actor["class"] == "StaticMeshActor" and mesh_name(actor)]
    min_x, max_x, min_y, max_y = -109.0, 56.0, 43.0, 312.0
    scale, left, top = 4.2, 70, 75
    width = int((max_x-min_x)*scale + 2*left)
    height = int((max_y-min_y)*scale + 2*top)

    def xy(point):
        return left+(point[0]-min_x)*scale, top+(max_y-point[1])*scale

    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {width} {height}" '
             f'width="{width}" height="{height}">',
             '<rect width="100%" height="100%" fill="#f8fafc"/>',
             '<text x="70" y="37" font-size="19" font-family="sans-serif" fill="#1e293b">'
             'IndustrialArea UE road graph — both copies as one park</text>',
             '<text x="70" y="56" font-size="12" font-family="sans-serif" fill="#475569">'
             'UE world XY (m); actual lane counts; turn paths require vehicle calibration</text>']
    for actor in roads:
        x, y = center(actor)
        dx, dy = extents(actor)
        px, py = xy((x-dx, y+dy))
        fill = "#cbd5e1" if mesh_name(actor) not in JUNCTION_MESHES else "#94a3b8"
        parts.append(f'<rect x="{px:.2f}" y="{py:.2f}" width="{2*dx*scale:.2f}" '
                     f'height="{2*dy*scale:.2f}" fill="{fill}" fill-opacity="0.65" '
                     f'stroke="#64748b" stroke-width="0.4"/>')
    for item in graph["edges"]:
        points = [xy(point) for point in item["centerline"]]
        value = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
        road = item["edge_id"].startswith("road_")
        color = "#075985" if road else "#a855f7"
        parts.append(f'<polyline points="{value}" fill="none" stroke="{color}" '
                     f'stroke-opacity="{0.72 if road else 0.24}" stroke-width="{2 if road else 0.9}"/>')
        if road:
            middle = item["centerline"][len(item["centerline"])//2]
            a, b = item["centerline"][0], item["centerline"][-1]
            ux, uy = (b[0]-a[0])/item_length(a, b), (b[1]-a[1])/item_length(a, b)
            tip = (middle[0]+ux*1.3, middle[1]+uy*1.3)
            p1 = (middle[0]-ux*.8-uy*.65, middle[1]-uy*.8+ux*.65)
            p2 = (middle[0]-ux*.8+uy*.65, middle[1]-uy*.8-ux*.65)
            triangle = " ".join(f"{x:.1f},{y:.1f}" for x, y in map(xy, (tip, p1, p2)))
            parts.append(f'<polygon points="{triangle}" fill="#075985"/>')
    for key, junction in survey["junctions"].items():
        x, y = xy(junction["center_m"])
        parts.append(f'<text x="{x+5:.1f}" y="{y-4:.1f}" font-size="10" '
                     f'font-family="monospace" fill="#0f172a">{escape(key)}</text>')
    for item in spawns["spawns"]:
        x, y = xy(item["ue_position_m"][:2])
        fill = "#dc2626" if item["inside_junction_by_center"] else "#f59e0b" if not item["ready_for_roadgraph_spawn"] else "#16a34a"
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="3.1" fill="{fill}">'
                     f'<title>{escape(item["vehicle_id"])}</title></circle>')
    parts.append('<text x="70" y="'+str(height-34)+'" font-size="12" font-family="sans-serif" fill="#475569">'
                 'Blue: directed road lanes  ·  Purple: turn paths  ·  Green: on-lane vehicle  ·  Orange: off-lane  ·  Red: in junction</text>')
    parts.append('</svg>')
    path.write_text("\n".join(parts)+"\n", encoding="utf-8")


def item_length(a, b):
    return hypot(b[0]-a[0], b[1]-a[1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, default=ROOT/"runs/ue_scene/scene.json")
    args = parser.parse_args()
    snapshot = json.loads(args.snapshot.read_text(encoding="utf-8"))
    graph, controls, survey, spawns = make_graph(snapshot)
    targets = ((ROOT/"config/topology/industrialarea_ue.json", graph),
               (ROOT/"config/traffic/industrialarea_ue_controls.json", controls),
               (ROOT/"config/traffic/industrialarea_ue_spawns.json", spawns),
               (ROOT/"config/topology/industrialarea_ue_survey.json", survey))
    for path, value in targets:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    write_svg(snapshot, graph, survey, spawns, ROOT/"docs/industrialarea_ue_roadgraph.svg")
    print(f"IndustrialArea RoadGraph: {len(graph['nodes'])} nodes, {len(graph['edges'])} edges, "
          f"{len(graph['movements'])} movements, {survey['junction_count']} junctions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
