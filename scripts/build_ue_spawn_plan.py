#!/usr/bin/env python3
"""Place 24 configured trucks on four painted east/west road lanes."""

import json
from pathlib import Path
from statistics import median


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    survey = json.loads((ROOT/"config/topology/industrialarea_ue_survey.json").read_text())
    current = json.loads((ROOT/"config/traffic/industrialarea_ue_spawns.json").read_text())["spawns"]
    if len(current) != 24:
        raise ValueError("expected 24 configured UE vehicles")
    names = [item["vehicle_id"] for item in current]
    if names != sorted(names):
        raise ValueError("vehicle IDs must be sorted")
    west = survey["junctions"]["j_0_0"]
    centre = survey["junctions"]["j_0_1"]
    start = west["center_m"][0]+west["half_extent_m"][0]
    end = centre["center_m"][0]-centre["half_extent_m"][0]
    # The current fleet includes a few yawed vehicles; their world AABBs are
    # larger than the chassis. All planned headings are axis-aligned.
    truck_length = median(2*item["pawn_half_extent_m"][0] for item in current)
    truck_width = median(2*item["pawn_half_extent_m"][1] for item in current)
    body_margin = .4
    minimum_x = start+truck_length/2+body_margin
    maximum_x = end-truck_length/2-body_margin
    step = (maximum_x-minimum_x)/5
    if step < truck_length+2.0:
        raise ValueError("six trucks cannot fit safely between the two junctions")
    x_positions = [round(minimum_x+i*step, 4) for i in range(6)]
    targets = []
    groups = ((0, 0, "west"), (6, 2, "west"), (12, 1, "east"), (18, 3, "east"))
    for offset, row, direction in groups:
        y = survey["junctions"][f"j_{row}_0"]["center_m"][1]
        y += survey["lane_offset_m"] if direction == "east" else -survey["lane_offset_m"]
        xs = x_positions if direction == "east" else list(reversed(x_positions))
        for index in range(6):
            source = current[offset+index]
            x = xs[index]
            z = source["ue_position_m"][2]
            target = {"vehicle_id": source["vehicle_id"],
                      "source_ue_position_m": source["ue_position_m"],
                      "target_ue_position_m": [x, round(y, 4), z],
                      "target_ue_yaw_deg": 0 if direction == "east" else 180,
                      "road_edge": f"road_j_{row}_0_j_{row}_1" if direction == "east" else f"road_j_{row}_1_j_{row}_0"}
            targets.append(target)
    # Rectangles are axis-aligned because every target heading is 0 or 180 deg.
    min_clearance = float("inf")
    for index, a in enumerate(targets):
        for b in targets[index+1:]:
            dx = abs(a["target_ue_position_m"][0]-b["target_ue_position_m"][0])-truck_length
            dy = abs(a["target_ue_position_m"][1]-b["target_ue_position_m"][1])-truck_width
            if dx < 0 and dy < 0:
                raise ValueError(f"target overlap: {a['vehicle_id']} and {b['vehicle_id']}")
            if dy < 0:
                min_clearance = min(min_clearance, dx)
            if dx < 0:
                min_clearance = min(min_clearance, dy)
    if min_clearance < 2.0:
        raise ValueError("vehicle clearance below 2.0 m")
    result = {"description": "24-truck road-lane spawn proposal; no UE setting or live vehicle is modified by this file",
              "coordinate_frame": "UE absolute world metres; convert with each session's AirSim world origin",
              "truck_length_m_from_pawn_bounds": round(truck_length, 3),
              "truck_width_m_from_pawn_bounds": round(truck_width, 3),
              "minimum_body_clearance_m": round(min_clearance, 3),
              "vehicles_per_lane": 6,
              "targets": targets}
    path = ROOT/"config/traffic/industrialarea_ue_spawn_plan.json"
    path.write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    print(f"UE spawn plan: {len(targets)} vehicles on 4 lanes, minimum body clearance {min_clearance:.2f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
