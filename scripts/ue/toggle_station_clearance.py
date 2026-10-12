"""Temporarily change selected small-prop collision in the running PIE world."""
import json
from pathlib import Path

import unreal

root = Path(__file__).resolve().parents[2]
out = root / "runs/ue_scene/station_siting"
request = json.loads((out / "toggle_station_clearance_request.json").read_text())
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
results = []
for spec in request["actors"]:
    matches = []
    for actor in actors:
        if actor.get_name() != spec["name"]:
            continue
        loc = actor.get_actor_location()
        x, y = (loc.x + origin.x) / 100, (loc.y + origin.y) / 100
        if (x - spec["xy_m"][0]) ** 2 + (y - spec["xy_m"][1]) ** 2 < 1:
            matches.append(actor)
    if len(matches) != 1:
        raise RuntimeError("Expected one small prop for " + str(spec))
    actor = matches[0]
    before = actor.get_actor_enable_collision()
    actor.set_actor_enable_collision(request["enabled"])
    results.append({"name": actor.get_name(), "before": before,
                    "after": actor.get_actor_enable_collision()})
(out / "toggle_station_clearance_result.json").write_text(json.dumps({
    "persistent_map_changed": False, "results": results}, indent=2) + "\n")
