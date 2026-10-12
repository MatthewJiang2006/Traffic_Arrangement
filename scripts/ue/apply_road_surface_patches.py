"""Save surveyed collision-only seam bridges into the backed-up UE editor map.

Run at editor startup with -ExecutePythonScript, before starting PIE.
"""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
CONFIG = json.loads((ROOT/"config/traffic/industrialarea_ue_surface_patches.json").read_text(encoding="utf-8"))
world = unreal.EditorLevelLibrary.get_editor_world()
if world is None or world.get_name() != "IndustrialArea（复件）":
    raise RuntimeError("Expected the IndustrialArea（复件） editor map")
if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    raise RuntimeError("Stop PIE before editing road surface patches")
if not (ROOT/"runs/ue_scene/IndustrialArea（复件）_before_seam_patch.umap").exists():
    raise RuntimeError("Original map backup is required before patching")
if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
    raise RuntimeError("Preserve existing unsaved map edits before patching")
if not unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）"):
    raise RuntimeError("Cannot select the persistent map level")
origin = unreal.GameplayStatics.get_world_origin_location(world)
cube = unreal.load_object(None, "/Engine/BasicShapes/Cube.Cube")
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
results = []
for patch in CONFIG["patches"]:
    found = [a for a in actors if str(patch["label"]) in [str(t) for t in a.tags]]
    if len(found) > 1:
        raise RuntimeError("duplicate road surface patch")
    if found:
        actor = found[0]
    else:
        p = patch["center_m"]
        position = unreal.Vector(p[0]*100-origin.x, p[1]*100-origin.y, p[2]*100-origin.z)
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor, position)
        if actor is None:
            raise RuntimeError("failed to spawn road surface patch")
    component = actor.get_components_by_class(unreal.StaticMeshComponent)[0]
    component.set_mobility(unreal.ComponentMobility.MOVABLE)
    component.set_static_mesh(cube)
    actor.set_actor_scale3d(unreal.Vector(*patch["size_m"]))
    component.set_collision_profile_name("BlockAll")
    component.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    component.set_mobility(unreal.ComponentMobility.STATIC)
    actor.set_actor_hidden_in_game(True)
    actor.set_actor_label(patch["label"])
    actor.set_editor_property("tags", [unreal.Name("TrafficRoadSurfacePatch"), unreal.Name(patch["label"])])
    component.set_editor_property("cast_shadow", False)
    results.append({"label": patch["label"], "actor": actor.get_path_name(),
                    "center_ue_m": patch["center_m"], "size_m": patch["size_m"]})
if not unreal.EditorLoadingAndSavingUtils.save_map(world, "/Game/IndustrialArea/Map/IndustrialArea（复件）"):
    raise RuntimeError("Road seam patch map save failed")
output = ROOT/"runs/ue_scene/road_surface_patches.json"
output.write_text(json.dumps({"world": world.get_path_name(), "saved": True, "patches": results}, indent=2), encoding="utf-8")
unreal.log("Saved two road seam collision patches in the persistent map")
