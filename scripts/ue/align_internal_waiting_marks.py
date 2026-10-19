"""Align noncolliding parking markings to validated parking orientations."""
import json,math
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/internal_waiting_access/mark_alignment.json'
handle=None
def apply(delta):
 global handle
 if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):return
 unreal.unregister_slate_post_tick_callback(handle)
 try:
  if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():raise RuntimeError('Unrelated unsaved map edits')
  world=unreal.EditorLevelLibrary.get_editor_world()
  if world.get_name()!='IndustrialArea（复件）':raise RuntimeError('Unexpected map')
  origin=unreal.GameplayStatics.get_world_origin_location(world)
  sites={s['id']:s for s in json.loads((ROOT/'config/planning/ue_internal_waiting_expansion.json').read_text())['sites']}
  changed=[]
  for a in unreal.EditorLevelLibrary.get_all_level_actors():
   label=a.get_actor_label()
   if label.startswith(('TrafficInternalWaitingMark_Q35_','TrafficInternalWaitingLabel_Q35')):
    unreal.EditorLevelLibrary.destroy_actor(a);changed.append(label);continue
   if not label.startswith(('TrafficInternalWaitingMark_','TrafficInternalWaitingLabel_')):continue
   key=label.split('_')[1];s=sites.get(key)
   if not s:continue
   x,y=s['xy_m'];h=math.radians(s['heading_deg']);co,si=math.cos(h),math.sin(h);z=s['ground_z_m']+.015
   if '_end' in label:
    side=int(label.split('_end')[1]);x+=side*4.5*co;y+=side*4.5*si
   elif '_side' in label:
    side=int(label.split('_side')[1]);x-=side*1.65*si;y+=side*1.65*co
   a.set_actor_location(unreal.Vector(x*100-origin.x,y*100-origin.y,(z+(.02 if 'Label' in label else 0))*100-origin.z),False,True)
   a.set_actor_rotation(unreal.Rotator(pitch=90 if 'Label' in label else 0,yaw=s['heading_deg'],roll=0),False);changed.append(label)
  if not unreal.EditorLoadingAndSavingUtils.save_dirty_packages(True,False):raise RuntimeError('Map save failed')
  OUT.write_text(json.dumps(dict(status='PASS',changed=changed,collision_geometry_changed=False,camera_changed=False),indent=2)+'\n')
 except Exception as exc:OUT.write_text(json.dumps(dict(status='FAIL',error=str(exc)))+'\n')
handle=unreal.register_slate_post_tick_callback(apply)
unreal.EditorLevelLibrary.editor_end_play()
