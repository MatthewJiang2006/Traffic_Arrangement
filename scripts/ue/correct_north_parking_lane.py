"""Move only the newly added north exit-road buffer to the legal left-turn lane."""
import json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'runs/ue_scene/parking_expansion';handle=None

def resume(delta):
 global handle
 unreal.unregister_slate_post_tick_callback(handle);handle=None
 unreal.EditorLevelLibrary.editor_play_simulate()

def apply(delta):
 global handle
 if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):return
 if handle is not None:unreal.unregister_slate_post_tick_callback(handle);handle=None
 try:
  if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():raise RuntimeError('Unrelated unsaved map edits')
  world=unreal.EditorLevelLibrary.get_editor_world();origin=unreal.GameplayStatics.get_world_origin_location(world)
  actors=unreal.EditorLevelLibrary.get_all_level_actors()
  rows=[a for a in actors if 'TrafficParkingRamp_N_98' in [str(t) for t in a.tags]]
  if len(rows)!=1:raise RuntimeError('Expected new parking ramp')
  a=rows[0];p=a.get_actor_location();old=[(p.x+origin.x)/100,(p.y+origin.y)/100]
  if abs(old[0]+97.8937)>.01:raise RuntimeError('Unexpected ramp position')
  p.x+=380;a.set_actor_location(p,False,True);a.set_actor_label('TrafficParkingRamp_N_94');a.set_editor_property('tags',[unreal.Name('TrafficParkingRamp_N_94')])
  unreal.EditorLevelLibrary.set_current_level_by_name('IndustrialArea_Copy')
  if not unreal.EditorLevelLibrary.save_current_level():raise RuntimeError('Save failed')
  unreal.EditorLevelLibrary.set_current_level_by_name('IndustrialArea（复件）')
  result=dict(status='PASS',saved=True,moved_new_ramp_only=True,old_xy_m=old,new_xy_m=[-94.0937,old[1]],camera_changed=False)
 except Exception as exc:result=dict(status='FAIL',error=str(exc))
 (OUT/'north_lane_correction.json').write_text(json.dumps(result,indent=2)+'\n')
 if result['status']=='PASS':handle=unreal.register_slate_post_tick_callback(resume)

handle=unreal.register_slate_post_tick_callback(apply)
if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):unreal.EditorLevelLibrary.editor_end_play()
