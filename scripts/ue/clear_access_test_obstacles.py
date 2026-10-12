"""Remove two diagnosed small walls from backed-up editor map and active PIE."""
import json
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/ue_access_connections'
request=json.loads((OUT/'obstacle_removal_request.json').read_text())
result=dict(status='FAIL',removed=[],camera_changed=False)
try:
    editor=unreal.load_object(None,'/Game/IndustrialArea/Map/IndustrialArea（复件）.IndustrialArea（复件）')
    if editor.get_name()!='IndustrialArea（复件）':raise RuntimeError('Wrong editor map')
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():raise RuntimeError('Unsaved user map edits')
    if not Path(request['backup']).exists():raise RuntimeError('Map backup missing')
    pie=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
    targets=[]
    for world in [editor,pie]:
        actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
        for name in ['SM_Block_Wall_57','SM_Block_Wall_68']:
            a=next(a for a in actors if a.get_name()==name)
            centre,half=a.get_actor_bounds(False)
            if max(half.x,half.y)>250 or half.z>250:raise RuntimeError('Not a small obstacle '+name)
            targets.append((world,a))
    for world,a in targets:
        name=a.get_name()
        if world==editor:
            a.destroy_actor()
        else:a.destroy_actor()
        result['removed'].append(dict(world=world.get_path_name(),actor=name))
    if not unreal.EditorLoadingAndSavingUtils.save_map(editor,'/Game/IndustrialArea/Map/IndustrialArea（复件）'):
        raise RuntimeError('Map save failed')
    result.update(status='PASS',backup=request['backup'],map_saved=True,pie_restarted=False)
except Exception as exc:result['error']=repr(exc)
(OUT/'obstacle_removal.json').write_text(json.dumps(result,indent=2)+'\n')
