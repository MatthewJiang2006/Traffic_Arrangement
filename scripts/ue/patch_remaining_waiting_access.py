"""Clear four surveyed fence pieces and bridge the inner waiting-berth curbs."""
import json
import math
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/"runs/ue_scene/station_siting"
BACKUPS=[OUT/"IndustrialArea_before_remaining_waiting_access.umap",
         OUT/"IndustrialArea_Copy_before_remaining_waiting_access.umap"]
PROPS=[("SM_Block_Wall_43",-80.035,192.338),("SM_Block_Wall_57",-79.994,195.651),
       ("SM_Block_Wall_65",-80.084,253.841),("SM_Block_Wall_68",-80.084,243.774)]
handle=None


def apply(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle=None
    world=unreal.EditorLevelLibrary.get_editor_world()
    if world.get_name()!="IndustrialArea（复件）":
        raise RuntimeError("Expected parent map")
    if not all(p.exists() for p in BACKUPS):
        raise RuntimeError("Both map backups required")
    origin=unreal.GameplayStatics.get_world_origin_location(world)
    actors=unreal.EditorLevelLibrary.get_all_level_actors()
    # Continue the known four removals if a prior run stopped before saving.
    resume=all(not any(a.get_name()==name and abs((a.get_actor_location().y+origin.y)/100-y)<.1
                      for a in actors) for name,x,y in PROPS)
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages() and not resume:
        raise RuntimeError("Unsaved edits unrelated to the interrupted fence removal")
    matched=[]
    removed=[]
    for name,x,y in PROPS:
        hits=[a for a in actors if a.get_name()==name and
              abs((a.get_actor_location().x+origin.x)/100-x)<.1 and
              abs((a.get_actor_location().y+origin.y)/100-y)<.1]
        if not hits and resume:
            removed.append(name)
            continue
        if len(hits)!=1 or not name.startswith("SM_Block_Wall_"):
            raise RuntimeError("Expected surveyed fence: "+name)
        extent=hits[0].get_actor_bounds(False)[1]
        if max(extent.x,extent.y,extent.z)>250:
            raise RuntimeError("Fence exceeds 5 m size: "+name)
        matched.append(hits[0])
    for actor in matched:
        removed.append(actor.get_name())
        if not unreal.EditorLevelLibrary.destroy_actor(actor):
            raise RuntimeError("Cannot remove "+actor.get_name())
    actors=unreal.EditorLevelLibrary.get_all_level_actors()
    cube=unreal.load_object(None,"/Engine/BasicShapes/Cube.Cube")
    material=next(a for a in actors if a.get_name()=="SM_Road_70").get_components_by_class(unreal.StaticMeshComponent)[0].get_material(0)
    ramps=[]
    for site,level,y in (("Q9Q10","IndustrialArea（复件）",119),("Q11Q12","IndustrialArea_Copy",247)):
        if not unreal.EditorLevelLibrary.set_current_level_by_name(level):
            raise RuntimeError("Cannot select "+level)
        for suffix,x,z,length,rise in (("RoadToCurb",-14.5,1.185,3.2,-.15),
                                       ("CurbToHardstand",-19.5,1.17,4.5,.176)):
            tag="TrafficWaiting"+site+suffix
            existing=[a for a in actors if tag in [str(t) for t in a.tags]]
            if existing:
                if len(existing)!=1:
                    raise RuntimeError("Duplicate ramp: "+tag)
                loc=existing[0].get_actor_location()
                if max(abs((loc.x+origin.x)/100-x),abs((loc.y+origin.y)/100-y),abs((loc.z+origin.z)/100-z))>.01:
                    raise RuntimeError("Existing ramp position differs: "+tag)
                scale=existing[0].get_actor_scale3d()
                if max(abs(scale.x-length),abs(scale.y-48),abs(scale.z-.04))>.001:
                    raise RuntimeError("Existing ramp size differs: "+tag)
                ramps.append(dict(tag=tag,level=level,center_m=[x,y,z],size_m=[length,48,.04]))
                continue
            actor=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor,
                unreal.Vector(x*100-origin.x,y*100-origin.y,z*100-origin.z),
                unreal.Rotator(pitch=math.degrees(math.atan2(rise,length)),yaw=0,roll=0))
            mesh=actor.get_components_by_class(unreal.StaticMeshComponent)[0]
            mesh.set_mobility(unreal.ComponentMobility.MOVABLE)
            mesh.set_static_mesh(cube)
            actor.set_actor_scale3d(unreal.Vector(length,48,.04))
            mesh.set_collision_profile_name("BlockAll")
            mesh.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
            mesh.set_material(0,material)
            mesh.set_mobility(unreal.ComponentMobility.STATIC)
            actor.set_actor_label(tag)
            actor.set_editor_property("tags",[unreal.Name(tag)])
            ramps.append(dict(tag=tag,level=level,center_m=[x,y,z],size_m=[length,48,.04]))
        if not unreal.EditorLevelLibrary.save_current_level():
            raise RuntimeError("Save failed: "+level)
    unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）")
    (OUT/"remaining_waiting_access_patch.json").write_text(json.dumps(dict(saved=True,
        removed_small_props=removed,ramps=ramps,backups=[str(p) for p in BACKUPS]),indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle=unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
