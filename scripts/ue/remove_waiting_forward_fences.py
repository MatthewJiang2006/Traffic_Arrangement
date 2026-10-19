"""Remove exactly two surveyed small fences after map backups."""
import json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/ue_scene/station_siting'
handle=None

def apply(delta):
    global handle
    if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle=None
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError('Unsaved map edits')
    backups=[OUT/'IndustrialArea_before_waiting_forward_fence_removal.umap',OUT/'IndustrialArea_Copy_before_waiting_forward_fence_removal.umap']
    if not all(p.exists() for p in backups):
        raise RuntimeError('Both backups required')
    world=unreal.EditorLevelLibrary.get_editor_world()
    origin=unreal.GameplayStatics.get_world_origin_location(world)
    actors=unreal.EditorLevelLibrary.get_all_level_actors()
    matched=[]
    for y in (119.308,247.332):
        hits=[a for a in actors if a.get_name()=='SM_Fence_1' and abs((a.get_actor_location().x+origin.x)/100+20.335)<.01 and abs((a.get_actor_location().y+origin.y)/100-y)<.01]
        if len(hits)!=1 or max(hits[0].get_actor_bounds(False)[1].x,hits[0].get_actor_bounds(False)[1].y,hits[0].get_actor_bounds(False)[1].z)>200:
            raise RuntimeError('Surveyed small fence mismatch')
        matched.append(hits[0])
    removed=[a.get_path_name() for a in matched]
    for actor in matched:
        if not unreal.EditorLevelLibrary.destroy_actor(actor):
            raise RuntimeError('Fence removal failed')
    for level in ('IndustrialArea（复件）','IndustrialArea_Copy'):
        if not unreal.EditorLevelLibrary.set_current_level_by_name(level) or not unreal.EditorLevelLibrary.save_current_level():
            raise RuntimeError('Map save failed')
    unreal.EditorLevelLibrary.set_current_level_by_name('IndustrialArea（复件）')
    (OUT/'waiting_forward_fence_removal.json').write_text(json.dumps(dict(saved=True,removed=removed,backups=[str(p) for p in backups]),indent=2)+'\n')

if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle=unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
