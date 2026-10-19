"""Mirror dynamically validated W1 reverse-entry ramps in upper W2 copy."""
import json
import math
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"runs/ue_scene/station_siting"
BACKUP = OUT/"IndustrialArea_Copy_before_W2_reverse_ramps.umap"
SUBLEVEL = "IndustrialArea_Copy"
RAMPS = [
    ("TrafficW2RoadToCurb", 40.5, 252.0, 1.175, -math.degrees(math.atan2(.15, 4.5))),
    ("TrafficW2CurbToHardstand", 33.0, 252.0, 1.162, math.degrees(math.atan2(.176, 4.5))),
]
handle = None


def apply(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle = None
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None or world.get_name() != "IndustrialArea（复件）":
        raise RuntimeError("Wrong editor map")
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Unsaved map edits")
    if not BACKUP.exists():
        raise RuntimeError("W2 sublevel backup required")
    if not unreal.EditorLevelLibrary.set_current_level_by_name(SUBLEVEL):
        raise RuntimeError("Cannot select copied upper sublevel")
    prefix = "/Game/IndustrialArea/Map/IndustrialArea_Copy.IndustrialArea_Copy:PersistentLevel."
    actors = [a for a in unreal.ObjectIterator(unreal.Actor)
              if a.get_path_name().startswith(prefix)]
    for tag, *_ in RAMPS:
        if any(tag in [str(t) for t in a.tags] for a in actors):
            raise RuntimeError("W2 ramp already exists: " + tag)
    road = next((a for a in actors if a.get_name() == "SM_Road_63"), None)
    material = None
    if road is not None:
        components = road.get_components_by_class(unreal.StaticMeshComponent)
        if components:
            material = components[0].get_material(0)
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    cube = unreal.load_object(None, "/Engine/BasicShapes/Cube.Cube")
    saved = []
    for tag, x, y, z, pitch in RAMPS:
        loc = unreal.Vector(x*100-origin.x, y*100-origin.y, z*100-origin.z)
        ramp = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.StaticMeshActor, loc, unreal.Rotator(pitch=pitch, yaw=0, roll=0))
        if ramp is None:
            raise RuntimeError("Cannot create " + tag)
        mesh = ramp.get_components_by_class(unreal.StaticMeshComponent)[0]
        mesh.set_mobility(unreal.ComponentMobility.MOVABLE)
        mesh.set_static_mesh(cube)
        ramp.set_actor_scale3d(unreal.Vector(4.5, 8.0, .04))
        mesh.set_collision_profile_name("BlockAll")
        mesh.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
        mesh.set_mobility(unreal.ComponentMobility.STATIC)
        ramp.set_actor_label(tag)
        ramp.set_editor_property("tags", [unreal.Name(tag)])
        mesh.set_editor_property("cast_shadow", False)
        if material is not None:
            mesh.set_material(0, material)
        saved.append({"tag": tag, "center_m": [x, y, z],
                      "size_m": [4.5, 8.0, .04], "pitch_deg": pitch})
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError("W2 sublevel save failed")
    unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）")
    (OUT/"w2_reverse_ramps_patch.json").write_text(json.dumps({
        "saved": True, "backup": str(BACKUP), "sublevel": SUBLEVEL,
        "ramps": saved}, indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
