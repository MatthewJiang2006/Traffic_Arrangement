"""Save 13 parking outlines and four low curb buffers; preserve existing actors."""
import json,math,time
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/ue_scene/parking_expansion';OUT.mkdir(parents=True,exist_ok=True)
PLAN=json.loads((ROOT/'config/planning/ue_parking_expansion.json').read_text())
handle=None

def apply(delta):
 global handle
 if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):return
 if handle is not None:unreal.unregister_slate_post_tick_callback(handle);handle=None
 try:
  world=unreal.EditorLevelLibrary.get_editor_world()
  if world.get_name()!='IndustrialArea（复件）':raise RuntimeError('Expected parent map')
  if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():raise RuntimeError('Unrelated unsaved map edits; no mutation performed')
  if not all((OUT/('before_'+n)).exists() for n in ('IndustrialArea（复件）.umap','IndustrialArea_Copy.umap')):raise RuntimeError('Backups missing')
  origin=unreal.GameplayStatics.get_world_origin_location(world)
  actors=unreal.EditorLevelLibrary.get_all_level_actors()
  before={a.get_path_name():dict(location=list(a.get_actor_location().to_tuple()),scale=list(a.get_actor_scale3d().to_tuple()),rotation=list(a.get_actor_rotation().to_tuple())) for a in actors}
  cube=unreal.load_object(None,'/Engine/BasicShapes/Cube.Cube')
  road=next(a for a in actors if a.get_name()=='SM_Road_70').get_components_by_class(unreal.StaticMeshComponent)[0].get_material(0)
  created=[]
  def mesh_actor(tag,x,y,z,l,w,t,yaw=0,pitch=0,collision=False):
   if any(tag in [str(v) for v in a.tags] for a in actors):raise RuntimeError('Parking actor already exists '+tag)
   a=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(x*100-origin.x,y*100-origin.y,z*100-origin.z),unreal.Rotator(pitch=pitch,yaw=yaw,roll=0))
   c=a.get_components_by_class(unreal.StaticMeshComponent)[0];c.set_mobility(unreal.ComponentMobility.MOVABLE);c.set_static_mesh(cube)
   a.set_actor_scale3d(unreal.Vector(l,w,t))
   c.set_collision_profile_name('BlockAll' if collision else 'NoCollision')
   c.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS if collision else unreal.CollisionEnabled.NO_COLLISION)
   if collision:c.set_material(0,road)
   c.set_mobility(unreal.ComponentMobility.STATIC);a.set_actor_label(tag);a.set_editor_property('tags',[unreal.Name(tag)])
   created.append(dict(tag=tag,xy_m=[x,y],center_z_m=z,size_m=[l,w,t],collision=collision))
  for north,level in ((False,'IndustrialArea（复件）'),(True,'IndustrialArea_Copy')):
   if not unreal.EditorLevelLibrary.set_current_level_by_name(level):raise RuntimeError('Missing level '+level)
   # 4m ramps blend the 0.10m asphalt edge into 1m flat terrain.
   for x in ((-97.8937,-86.4937) if north else (-97.8936,-86.4936)):
    tag='TrafficParkingRamp_'+('N' if north else 'S')+'_'+str(round(abs(x)))
    mesh_actor(tag,x,305.75 if north else 48.475,1.035,4,4,.04,270 if north else 90,math.degrees(math.atan2(.11,4)),True)
   for s in PLAN['sites']:
    if (s['region']=='north_outer')!=north:continue
    x,y=s['xy_m']
    for j,dy in enumerate((-4.5,4.5)):mesh_actor('TrafficParkingMark_'+s['id']+'_end'+str(j),x,y+dy,1.008,3.3,.08,.01)
    for j,dx in enumerate((-1.65,1.65)):mesh_actor('TrafficParkingMark_'+s['id']+'_side'+str(j),x+dx,y,1.008,.08,9,.01)
    a=unreal.EditorLevelLibrary.spawn_actor_from_class(unreal.TextRenderActor,unreal.Vector(x*100-origin.x,y*100-origin.y,102-origin.z),unreal.Rotator(pitch=90,yaw=0,roll=0))
    c=a.get_components_by_class(unreal.TextRenderComponent)[0];c.set_text(s['id']);c.set_world_size(85);c.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    a.set_actor_label('TrafficParkingLabel_'+s['id'])
   if not unreal.EditorLevelLibrary.save_current_level():raise RuntimeError('Save failed '+level)
  unreal.EditorLevelLibrary.set_current_level_by_name('IndustrialArea（复件）')
  after={a.get_path_name():dict(location=list(a.get_actor_location().to_tuple()),scale=list(a.get_actor_scale3d().to_tuple()),rotation=list(a.get_actor_rotation().to_tuple())) for a in unreal.EditorLevelLibrary.get_all_level_actors()}
  changed=[k for k,v in before.items() if after.get(k)!=v]
  result=dict(status='PASS' if not changed else 'FAIL',saved=True,created=created,existing_actor_count=len(before),changed_existing_actors=changed,camera_changed=False)
 except Exception as exc:result=dict(status='FAIL',error=str(exc))
 (OUT/'map_patch.json').write_text(json.dumps(result,indent=2)+'\n')

if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
 handle=unreal.register_slate_post_tick_callback(apply);unreal.EditorLevelLibrary.editor_end_play()
else:apply(0)
