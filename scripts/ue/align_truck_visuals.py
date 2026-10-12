"""Offset visual-only BP_Truck actors so their visible tires touch the road."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
worlds = [w for w in unreal.ObjectIterator(unreal.World)
          if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name()]
if len(worlds) != 1:
    raise RuntimeError("Expected one active IndustrialArea PIE world")
world = worlds[0]
visuals = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
           if a.get_class().get_name() == "BP_Truck_C"]
if len(visuals) != 24:
    raise RuntimeError("Expected exactly 24 visual truck actors")
results = []
VISUAL_Z_OFFSET_CM = 21
for visual in visuals:
    parent = visual.get_attach_parent_actor()
    if not parent or parent.get_class().get_name() != "TrafficTruckPawn":
        raise RuntimeError("BP_Truck visual is not attached to TrafficTruckPawn")
    visual.set_actor_relative_location(unreal.Vector(0, 0, VISUAL_Z_OFFSET_CM), False, False)
    visual_position = visual.get_actor_location()
    parent_position = parent.get_actor_location()
    results.append({"visual": visual.get_name(), "vehicle": parent.get_name(),
                    "world_offset_cm": [visual_position.x-parent_position.x,
                                        visual_position.y-parent_position.y,
                                        visual_position.z-parent_position.z]})
output = ROOT/"runs/ue_scene/visual_alignment.json"
output.write_text(json.dumps({"count": len(results), "visual_z_offset_cm": VISUAL_Z_OFFSET_CM,
                              "vehicles": sorted(results, key=lambda row: row["vehicle"])}, indent=2),
                  encoding="utf-8")
unreal.log("Aligned 24 truck visuals with road surface")
