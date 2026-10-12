"""Bridge the complete surveyed raised curb strips, preserving buildings."""
import json,math
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/internal_waiting_access/curb_bridge_patch.json'
handle=None
def apply(delta):
    global handle
    if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):return
    if handle is not None:unreal.unregister_slate_post_tick_callback(handle);handle=None
    try:
        if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():raise RuntimeError('Unrelated unsaved map edits')
        world=unreal.EditorLevelLibrary.get_editor_world();origin=unreal.GameplayStatics.get_world_origin_location(world)
        actors=unreal.EditorLevelLibrary.get_all_level_actors()
        for a in actors:
            if a.get_actor_label().startswith(('TrafficInternalWaitingHump_','TrafficInternalWaitingRamp_')):
                unreal.EditorLevelLibrary.destroy_actor(a)
        cube=unreal.load_object(None,'/Engine/BasicShapes/Cube.Cube')
        material=next(a for a in actors if a.get_name()=='SM_Road_70').get_component_by_class(unreal.StaticMeshComponent).get_material(0)
        created=[]
        def mesh(name,x,y,z,l,w,yaw,pitch=0):
            a=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(x*100-origin.x,y*100-origin.y,z*100-origin.z),unreal.Rotator(pitch=pitch,yaw=yaw,roll=0))
            a.set_actor_label(name);a.set_editor_property('tags',[name])
            c=a.get_component_by_class(unreal.StaticMeshComponent);c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_static_mesh(cube);c.set_material(0,material)
            a.set_actor_scale3d(unreal.Vector(l,w,.04));c.set_collision_profile_name('BlockAll');c.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS);c.set_mobility(unreal.ComponentMobility.STATIC)
            created.append(name)
        for key,x,y,yaw,width in [('Q26',-42,215,270,14),('Q28',-42,87,270,14),('Q29',-22,271.024,90,8),('Q30',-22,143,90,8),('Q31',-35,131,270,8),('Q32',-35,259.024,270,8),('Q34',-82,173,0,8),('Q35',-16.5,301,180,12),('Q36',-16.5,57,180,8)]:
            co,si=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
            mesh('TrafficInternalWaitingRamp_'+key+'_plate',x,y,1.234,3.5,width,yaw)
            for side in (-1,1):
                mesh('TrafficInternalWaitingRamp_'+key+'_'+str(side),x+side*3.75*co,y+side*3.75*si,1.144,4,width,yaw,-side*math.degrees(math.atan2(.18,4)))
        if not unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True,False):raise RuntimeError('Map save failed')
        OUT.write_text(json.dumps(dict(status='PASS',created=created,large_buildings_changed=False,camera_changed=False),indent=2)+'\n')
    except Exception as exc:OUT.write_text(json.dumps(dict(status='FAIL',error=str(exc)))+'\n')
handle=unreal.register_slate_post_tick_callback(apply)
unreal.EditorLevelLibrary.editor_end_play()
