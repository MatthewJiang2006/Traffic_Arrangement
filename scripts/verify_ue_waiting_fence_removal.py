#!/usr/bin/env python3
"""Preserve passed waiting trials only after verified obstacle-only removal."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/'runs/ue_scene/station_siting'
before=json.loads((RAW/'scene_before_waiting_forward_fence_removal.json').read_text())
after=json.loads((ROOT/'runs/ue_scene/scene.json').read_text())
removed=set(json.loads((RAW/'waiting_forward_fence_removal.json').read_text())['removed'])
def static(scene):
    return {a['path']:a for a in scene['actors'] if a['meshes'] and a['class'] not in ('TrafficTruckPawn','BP_Truck_C','CameraActor')}
a,b=static(before),static(after)
missing=set(a)-set(b)
changed=[k for k in set(a)&set(b) if a[k]!=b[k]]
verified=(missing==removed and not set(b)-set(a) and not changed and all(k.endswith(':PersistentLevel.SM_Fence_1') and max(a[k]['bounds_extent_cm'])<200 for k in missing))
report=dict(obstacles_only_removed_verified=verified,removed_actor_paths=sorted(missing),changed_actor_paths=sorted(changed),added_actor_paths=sorted(set(b)-set(a)),before_map_sha256={},after_map_sha256={},cases={},read_only=True)
maps=Path('/home/song03/Traffic_ue4/Content/IndustrialArea/Map')
for name,backup in (('IndustrialArea（复件）.umap','IndustrialArea_before_waiting_forward_fence_removal.umap'),('IndustrialArea_Copy.umap','IndustrialArea_Copy_before_waiting_forward_fence_removal.umap')):
    report['before_map_sha256'][name]=hashlib.sha256((RAW/backup).read_bytes()).hexdigest()
    report['after_map_sha256'][name]=hashlib.sha256((maps/name).read_bytes()).hexdigest()
for path in RAW.glob('waiting_access_*.json'):
    source=json.loads(path.read_text())
    if source.get('status')!='PASS' or source.get('map_sha256')!=report['before_map_sha256']:
        continue
    supports=[]
    for phase in source['phases'].values():
        for sample in phase['samples']:
            supports.extend(sample.get('wheel_surfaces') or [])
    supported_by_removed=any(s in removed or str(s).split('.')[-1]=='SM_Fence_1' for s in supports if s)
    report['cases'][path.stem]=dict(clearance_preserved=verified and not supported_by_removed,removed_actor_used_as_support=supported_by_removed)
(RAW/'waiting_fence_removal_preservation.json').write_text(json.dumps(report,indent=2)+'\n')
print('obstacle-only removal verified:',verified,'preserved:',list(report['cases']))
if not verified:
    raise SystemExit(1)
