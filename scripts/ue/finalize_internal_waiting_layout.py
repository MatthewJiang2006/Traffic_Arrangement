"""Replace low buffers with curb humps and mark internal candidate bays."""
import json,math
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/internal_waiting_access/layout_patch.json'
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
        actors=unreal.EditorLevelLibrary.get_all_level_actors();removed=[]
        for a in actors:
            label=a.get_actor_label();loc=a.get_actor_location();x,y=(loc.x+origin.x)/100,(loc.y+origin.y)/100
            foliage=a.get_name().startswith('SM_Leaf_') and -55<x<-30 and (75<y<95 or 203<y<223)
            old=label.startswith(('TrafficInternalWaitingBuffer_','TrafficParkingMark_','TrafficParkingLabel_','TrafficParkingRamp_'))
            if foliage or old:
                removed.append(dict(path=a.get_path_name(),label=label))
                unreal.EditorLevelLibrary.destroy_actor(a)
        cube=unreal.load_object(None,'/Engine/BasicShapes/Cube.Cube')
        white=unreal.load_object(None,'/Engine/BasicShapes/BasicShapeMaterial.BasicShapeMaterial')
        created=[]
        def mesh(name,x,y,z,l,w,t,yaw,pitch=0,collision=False):
            a=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(x*100-origin.x,y*100-origin.y,z*100-origin.z),unreal.Rotator(pitch=pitch,yaw=yaw,roll=0))
            a.set_actor_label(name);a.set_editor_property('tags',[name]);a.set_actor_scale3d(unreal.Vector(l,w,t))
            c=a.get_component_by_class(unreal.StaticMeshComponent);c.set_static_mesh(cube);c.set_material(0,white)
            c.set_collision_profile_name('BlockAll' if collision else 'NoCollision');created.append(name)
        for key,x,y,yaw,width in [('Q26',-45,215.5,270,8),('Q28',-45,87.5,270,8),('Q36',-15,57,180,8),('Q35',-14.5,301,180,12)]:
            co,si=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
            for side in (-1,1):
                mesh('TrafficInternalWaitingHump_'+key+'_'+str(side),x+side*2*co,y+side*2*si,1.137,4,width,.04,yaw,-side*math.degrees(math.atan2(.16,4)),True)
        plan=json.loads((ROOT/'config/planning/ue_internal_waiting_expansion.json').read_text())
        for s in plan['sites']:
            x,y=s['xy_m'];h=math.radians(s['heading_deg']);co,si=math.cos(h),math.sin(h);z=s['ground_z_m']+.015
            for side in (-1,1):
                mesh('TrafficInternalWaitingMark_'+s['id']+'_end'+str(side),x+side*4.5*co,y+side*4.5*si,z,.08,3.3,.01,s['heading_deg'])
                mesh('TrafficInternalWaitingMark_'+s['id']+'_side'+str(side),x-side*1.65*si,y+side*1.65*co,z,9,.08,.01,s['heading_deg'])
            a=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.TextRenderActor,unreal.Vector(x*100-origin.x,y*100-origin.y,(z+.02)*100-origin.z),unreal.Rotator(pitch=90,yaw=s['heading_deg'],roll=0))
            a.set_actor_label('TrafficInternalWaitingLabel_'+s['id']);c=a.get_component_by_class(unreal.TextRenderComponent)
            c.set_text(s['id']);c.set_world_size(70);c.set_horizontal_alignment(unreal.HorizTextAligment.EHTA_CENTER);created.append(a.get_actor_label())
        if not unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True,False):raise RuntimeError('Map save failed')
        OUT.write_text(json.dumps(dict(status='PASS',removed=removed,created=created,large_buildings_changed=False,camera_changed=False),indent=2)+'\n')
    except Exception as exc:OUT.write_text(json.dumps(dict(status='FAIL',error=str(exc)))+'\n')
handle=unreal.register_slate_post_tick_callback(apply)
unreal.EditorLevelLibrary.editor_end_play()
