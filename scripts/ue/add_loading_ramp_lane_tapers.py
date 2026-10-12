"""Blend the two road-side loading ramps into their adjacent longitudinal lane."""
import json
import math
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"runs/ue_scene/station_siting"
SITES = [
    ("W1", "IndustrialArea（复件）", 124.0, "IndustrialArea_before_loading_ramp_lane_tapers.umap"),
    ("W2", "IndustrialArea_Copy", 252.0, "IndustrialArea_Copy_before_loading_ramp_lane_tapers.umap"),
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
    for _,_,_,backup in SITES:
        if not (OUT/backup).exists():
            raise RuntimeError("Both loading-ramp map backups required")
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    cube = unreal.load_object(None,"/Engine/BasicShapes/Cube.Cube")
    saved = []
    for site,level,y,backup in SITES:
        existing = [a for a in actors if f"Traffic{site}RoadToCurb" in [str(t) for t in a.tags]]
        if len(existing) != 1:
            raise RuntimeError("Expected exactly one road ramp at "+site)
        loc = existing[0].get_actor_location()
        if abs((loc.x+origin.x)/100-40.5)>.1 or abs((loc.y+origin.y)/100-y)>.1:
            raise RuntimeError("Road ramp position changed at "+site)
        material = existing[0].get_components_by_class(unreal.StaticMeshComponent)[0].get_material(0)
        if not unreal.EditorLevelLibrary.set_current_level_by_name(level):
            raise RuntimeError("Cannot select "+level)
        for side,dy,roll_sign in (("LowY",-6.0,-1),("HighY",6.0,1)):
            tag = f"Traffic{site}LaneTaper{side}"
            if any(tag in [str(t) for t in a.tags] for a in actors):
                raise RuntimeError("Taper already exists: "+tag)
            ramp = unreal.EditorLevelLibrary.spawn_actor_from_class(
                unreal.StaticMeshActor,
                unreal.Vector(40.5*100-origin.x,(y+dy)*100-origin.y,1.1275*100-origin.z),
                unreal.Rotator(pitch=-math.degrees(math.atan2(.15,4.5)),
                               roll=roll_sign*math.degrees(math.atan2(.095,4.0)),yaw=0))
            if ramp is None:
                raise RuntimeError("Cannot spawn "+tag)
            mesh = ramp.get_components_by_class(unreal.StaticMeshComponent)[0]
            mesh.set_mobility(unreal.ComponentMobility.MOVABLE)
            mesh.set_static_mesh(cube)
            ramp.set_actor_scale3d(unreal.Vector(4.5,4.0,.04))
            mesh.set_collision_profile_name("BlockAll")
            mesh.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
            mesh.set_mobility(unreal.ComponentMobility.STATIC)
            mesh.set_editor_property("cast_shadow",False)
            if material is not None:
                mesh.set_material(0,material)
            ramp.set_actor_label(tag)
            ramp.set_editor_property("tags",[unreal.Name(tag)])
            saved.append(dict(site=site,tag=tag,level=level,
                              center_m=[40.5,y+dy,1.1275],size_m=[4.5,4.0,.04]))
        if not unreal.EditorLevelLibrary.save_current_level():
            raise RuntimeError("Cannot save lane tapers in "+level)
    unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）")
    (OUT/"loading_ramp_lane_tapers.json").write_text(json.dumps(dict(
        saved=True,ramps=saved,backups=[str(OUT/s[3]) for s in SITES]),indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
