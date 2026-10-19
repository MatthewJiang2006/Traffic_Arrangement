"""Add internal curb buffers and remove interfering ground foliage only."""
import json,math
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/internal_waiting_access/map_patch.json'
handle=None
def apply(delta):
    global handle
    if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):return
    if handle is not None:unreal.unregister_slate_post_tick_callback(handle);handle=None
    try:
        if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():raise RuntimeError('Unrelated unsaved map edits')
        world=unreal.EditorLevelLibrary.get_editor_world()
        if world.get_name()!='IndustrialArea（复件）':raise RuntimeError('Unexpected editor map')
        origin=unreal.GameplayStatics.get_world_origin_location(world)
        actors=unreal.EditorLevelLibrary.get_all_level_actors()
        removed=[]
        for a in actors:
            if not a.get_name().startswith(('SM_Leaf_19','SM_Leaf_20')):continue
            v=a.get_actor_location();x,y=(v.x+origin.x)/100,(v.y+origin.y)/100
            if -55<x<-35 and (80<y<93 or 208<y<221):
                removed.append(dict(path=a.get_path_name(),xy_m=[x,y]))
                unreal.EditorLevelLibrary.destroy_actor(a)
        cube=unreal.load_object(None,'/Engine/BasicShapes/Cube.Cube')
        material=next(a for a in actors if a.get_name()=='SM_Road_70').get_components_by_class(unreal.StaticMeshComponent)[0].get_material(0)
        created=[]
        for tag,x,y,yaw in [('Q26',-45,215.5,270),('Q28',-45,87.5,270),('Q36',-16,57,180)]:
            name='TrafficInternalWaitingBuffer_'+tag
            if any(a.get_actor_label()==name for a in actors):raise RuntimeError('Buffer exists '+name)
            a=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(x*100-origin.x,y*100-origin.y,.9985*100-origin.z),unreal.Rotator(pitch=math.degrees(math.atan2(.11,8)),yaw=yaw,roll=0))
            a.set_actor_label(name);a.set_editor_property('tags',[name])
            a.set_actor_scale3d(unreal.Vector(8,6,.04))
            c=a.get_component_by_class(unreal.StaticMeshComponent);c.set_static_mesh(cube);c.set_material(0,material)
            c.set_collision_profile_name('BlockAll');created.append(name)
        if not unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True,False):raise RuntimeError('Map save failed')
        OUT.write_text(json.dumps(dict(status='PASS',removed_foliage=removed,created_buffers=created,large_buildings_changed=False,camera_changed=False),indent=2)+'\n')
    except Exception as exc:OUT.write_text(json.dumps(dict(status='FAIL',error=str(exc)))+'\n')
handle=unreal.register_slate_post_tick_callback(apply)
unreal.EditorLevelLibrary.editor_end_play()
