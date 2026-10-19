"""Extend only two known road-side ramps across the measured 1.15 m gaps."""
import json
import math
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
        unreal.unregister_slate_post_tick_callback(handle);handle=None
    world=unreal.EditorLevelLibrary.get_editor_world()
    if world.get_name()!='IndustrialArea（复件）' or unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError('Expected clean parent editor world')
    backups=[OUT/'IndustrialArea_before_waiting_ramp_seam_repair.umap',OUT/'IndustrialArea_Copy_before_waiting_ramp_seam_repair.umap']
    if not all(p.exists() for p in backups):raise RuntimeError('Backups required')
    origin=unreal.GameplayStatics.get_world_origin_location(world)
    actors=unreal.EditorLevelLibrary.get_all_level_actors();targets=[]
    for group,y,level in (('Q9Q10',119,'IndustrialArea（复件）'),('Q11Q12',247,'IndustrialArea_Copy')):
        tag='TrafficWaiting'+group+'RoadToCurb'
        found=[a for a in actors if tag in [str(t) for t in a.tags]]
        if len(found)!=1:raise RuntimeError('Expected one ramp '+tag)
        actor=found[0];loc=actor.get_actor_location();scale=actor.get_actor_scale3d()
        if max(abs((loc.x+origin.x)/100+14.5),abs((loc.y+origin.y)/100-y),abs(scale.x-3.2),abs(scale.y-48),abs(scale.z-.04))>.001:
            raise RuntimeError('Surveyed ramp changed '+tag)
        targets.append((actor,tag,y,level))
    rows=[]
    for actor,tag,y,level in targets:
        mesh=actor.get_components_by_class(unreal.StaticMeshComponent)[0]
        mesh.set_mobility(unreal.ComponentMobility.MOVABLE)
        actor.set_actor_location(unreal.Vector(-1510-origin.x,y*100-origin.y,116.75-origin.z),False,False)
        actor.set_actor_rotation(unreal.Rotator(pitch=-math.degrees(math.atan2(.175,4.8)),yaw=0,roll=0),False)
        actor.set_actor_scale3d(unreal.Vector(4.8,48,.04))
        mesh.set_mobility(unreal.ComponentMobility.STATIC)
        rows.append(dict(actor=tag,level=level,center_m=[-15.1,y,1.1675],size_m=[4.8,48,.04],rise_m=-.175,old_center_m=[-14.5,y,1.185],old_size_m=[3.2,48,.04]))
    for level in ('IndustrialArea（复件）','IndustrialArea_Copy'):
        if not unreal.EditorLevelLibrary.set_current_level_by_name(level) or not unreal.EditorLevelLibrary.save_current_level():raise RuntimeError('Save failed '+level)
    unreal.EditorLevelLibrary.set_current_level_by_name('IndustrialArea（复件）')
    (OUT/'waiting_ramp_seam_repair.json').write_text(json.dumps(dict(saved=True,modified=rows,removed_actors=[],backups=[str(p) for p in backups]),indent=2)+'\n')

if any('UEDPIE_' in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle=unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
