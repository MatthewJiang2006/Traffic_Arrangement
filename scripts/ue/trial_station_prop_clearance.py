"""Temporarily disable selected PIE prop collisions for access diagnosis."""
import json
from pathlib import Path
import runpy

import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "runs/ue_scene/station_siting"
request = json.loads((OUT / "prop_trial_request.json").read_text())
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
changed = []
prop_prefixes = ("SM_Fence_", "SM_Pole_", "SM_Barrel_", "SM_Block_",
                 "SM_Stop_Sign_", "BP_Meshes_")
try:
    for box in request.get("small_props_in_bounds", []):
        x0, y0, x1, y1 = box
        for actor in actors:
            if not actor.get_name().startswith(prop_prefixes):
                continue
            loc = actor.get_actor_location()
            x, y = (loc.x+origin.x)/100, (loc.y+origin.y)/100
            if not (x0 <= x <= x1 and y0 <= y <= y1):
                continue
            enabled = actor.get_actor_enable_collision()
            if enabled:
                actor.set_actor_enable_collision(False)
                changed.append((actor, enabled))
    for spec in request["props"]:
        matches = []
        for actor in actors:
            if actor.get_name() != spec["name"]:
                continue
            loc = actor.get_actor_location()
            x, y = (loc.x+origin.x)/100, (loc.y+origin.y)/100
            if (x-spec["xy_m"][0])**2+(y-spec["xy_m"][1])**2 < 4:
                matches.append(actor)
        if len(matches) != 1:
            raise RuntimeError("expected one prop "+str(spec)+", got "+str(len(matches)))
        actor = matches[0]
        enabled = actor.get_actor_enable_collision()
        if enabled:
            actor.set_actor_enable_collision(False)
            changed.append((actor, enabled))
    route_request = OUT / "route_request.json"
    route_request.write_text(json.dumps({
        "ids": request["ids"], "output": request.get("output", "temporary_prop_routes.json"),
        "max_expansions": 8000, "turn_deg": 20,
        "ground_tolerance_m": request.get("ground_tolerance_m", .08)}))
    try:
        runpy.run_path(str(ROOT / "scripts/ue/plan_warehouse_access.py"))
    finally:
        route_request.unlink()
finally:
    for actor, enabled in changed:
        actor.set_actor_enable_collision(enabled)
(OUT / "prop_trial_result.json").write_text(json.dumps({
    "read_only_persistent_map": True,
    "temporarily_disabled": [a.get_name() for a, _ in changed],
    "restored": all(actor.get_actor_enable_collision() == enabled for actor, enabled in changed),
    "route_output": request.get("output", "temporary_prop_routes.json")}, indent=2)+"\n")
