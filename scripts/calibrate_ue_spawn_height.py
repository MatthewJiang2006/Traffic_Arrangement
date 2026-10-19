#!/usr/bin/env python3
"""Set truck spawn height from visible tire geometry and measured road surfaces."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT/"config/traffic/industrialarea_ue_spawn_plan.json"
SURFACE = ROOT/"runs/ue_scene/spawn_surface_validation.json"


def main() -> int:
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    validation = json.loads(SURFACE.read_text(encoding="utf-8"))
    if not validation["all_road_surfaces"] or validation["vehicle_count"] != 24:
        raise ValueError("all 24 truck footprints must be on road meshes")
    measured = {row["vehicle_id"]: row for row in validation["vehicles"]}
    if set(measured) != {row["vehicle_id"] for row in plan["targets"]}:
        raise ValueError("surface validation does not match spawn plan")
    for target in plan["targets"]:
        row = measured[target["vehicle_id"]]
        x, y, _ = target["target_ue_position_m"]
        centre = row["samples"][1]
        if abs(centre["ue_xy_m"][0]-x) > .002 or abs(centre["ue_xy_m"][1]-y) > .002:
            raise ValueError(f"stale surface validation for {target['vehicle_id']}")
        levels = [sample["ue_surface_z_m"] for sample in row["samples"]]
        if max(levels)-min(levels) > .03:
            raise ValueError(f"uneven road surface under {target['vehicle_id']}")
        target["target_ue_position_m"][2] = row["target_actor_z_m_for_visual_tire_contact"]
    plan["vertical_placement"] = "UE actor origin calibrated to visible tire bottoms and settled SUV physics; visual actor relative Z is +21 cm"
    plan["visual_z_offset_cm"] = 21
    PLAN.write_text(json.dumps(plan, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print("24 truck heights calibrated against visible tires and UE road surfaces")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
