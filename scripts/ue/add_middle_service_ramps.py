"""Bridge the two measured curb steps at the W5/W6 service entrances."""
import json
import math
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/"runs/ue_scene/station_siting"
BACKUPS=[OUT/"IndustrialArea_before_middle_service_ramps.umap",
         OUT/"IndustrialArea_Copy_before_middle_service_ramps.umap"]
handle=None


def apply(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle=None
    world=unreal.EditorLevelLibrary.get_editor_world()
    if world.get_name()!="IndustrialArea（复件）" or unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Expected clean saved parent map")
    if not all(p.exists() for p in BACKUPS):
        raise RuntimeError("Both map backups required")
    origin=unreal.GameplayStatics.get_world_origin_location(world)
    actors=unreal.EditorLevelLibrary.get_all_level_actors()
    cube=unreal.load_object(None,"/Engine/BasicShapes/Cube.Cube")
    rows=[]
    for site,level,y in (("W5","IndustrialArea（复件）",173),("W6","IndustrialArea_Copy",184)):
        if not unreal.EditorLevelLibrary.set_current_level_by_name(level):
            raise RuntimeError("Cannot select "+level)
        road=next(a for a in actors if a.get_name()==("SM_Road_294" if site=="W5" else "SM_Road_87")
                  and abs((a.get_actor_location().y+origin.y)/100-y)<10)
        material=road.get_components_by_class(unreal.StaticMeshComponent)[0].get_material(0)
        for suffix,x,z,length,rise in (("RoadToCurb",-14.5,1.185,3.2,-.15),
                                       ("CurbToHardstand",-19.5,1.17,4.5,.176)):
            tag="Traffic"+site+suffix
            if any(tag in [str(t) for t in a.tags] for a in actors):
                raise RuntimeError("Already exists: "+tag)
            ramp=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor,
                unreal.Vector(x*100-origin.x,y*100-origin.y,z*100-origin.z),
                unreal.Rotator(pitch=math.degrees(math.atan2(rise,length)),yaw=0,roll=0))
            mesh=ramp.get_components_by_class(unreal.StaticMeshComponent)[0]
            mesh.set_mobility(unreal.ComponentMobility.MOVABLE)
            mesh.set_static_mesh(cube)
            ramp.set_actor_scale3d(unreal.Vector(length,5.5,.04))
            mesh.set_collision_profile_name("BlockAll")
            mesh.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
            mesh.set_material(0,material)
            mesh.set_mobility(unreal.ComponentMobility.STATIC)
            ramp.set_actor_label(tag)
            ramp.set_editor_property("tags",[unreal.Name(tag)])
            rows.append(dict(tag=tag,level=level,center_m=[x,y,z],size_m=[length,5.5,.04]))
        if not unreal.EditorLevelLibrary.save_current_level():
            raise RuntimeError("Save failed: "+level)
    unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）")
    (OUT/"middle_service_ramps.json").write_text(json.dumps(dict(saved=True,ramps=rows,
        backups=[str(p) for p in BACKUPS],removed_actors=[]),indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle=unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
