#!/usr/bin/env python3
"""Preserve passed access evidence only where complete recorded sweeps are unchanged."""
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];RAW=ROOT/'runs/ue_scene/station_siting'
after=json.loads((ROOT/'runs/ue_scene/scene.json').read_text())
current={n:hashlib.sha256((Path('/home/song03/Traffic_ue4/Content/IndustrialArea/Map')/n).read_bytes()).hexdigest() for n in ('IndustrialArea（复件）.umap','IndustrialArea_Copy.umap')}
sources=[]
for suffix in ('waiting_ramp_seam_repair','waiting_forward_fence_removal','remaining_waiting_access'):
    scene=RAW/f'scene_before_{suffix}.json'
    if not scene.exists():continue
    maps={n:hashlib.sha256((RAW/f'{stem}_before_{suffix}.umap').read_bytes()).hexdigest() for n,stem in (('IndustrialArea（复件）.umap','IndustrialArea'),('IndustrialArea_Copy.umap','IndustrialArea_Copy'))}
    sources.append((maps,json.loads(scene.read_text())))
def selected(scene,rectangles):
    rows=[]
    for a in scene['actors']:
        if not a['meshes'] or a['class'] in ('TrafficTruckPawn','BP_Truck_C','CameraActor'):continue
        x,y=a['bounds_origin_cm'][:2];hx,hy=a['bounds_extent_cm'][:2]
        if any(x/100+hx/100>=x0 and x/100-hx/100<=x1 and y/100+hy/100>=y0 and y/100-hy/100<=y1 for x0,y0,x1,y1 in rectangles):rows.append(a)
    return sorted(rows,key=lambda a:a['path'])
report=dict(after_map_sha256=current,cases={},read_only=True)
for path in [*RAW.glob('service_access_*.json'),*RAW.glob('waiting_access_*.json')]:
    r=json.loads(path.read_text())
    if r.get('status')!='PASS' or not r.get('vehicle_restored'):continue
    before=next((s for maps,s in sources if maps==r.get('map_sha256')),None)
    if before is None:continue
    rects=[]
    for p in r['phases'].values():
        for a,b in zip(p['path_xy_heading'],p['path_xy_heading'][1:]):rects.append([min(a[0],b[0])-6,min(a[1],b[1])-6,max(a[0],b[0])+6,max(a[1],b[1])+6])
    a,b=selected(before,rects),selected(after,rects)
    digest=lambda v:hashlib.sha256(json.dumps(v,sort_keys=True).encode()).hexdigest()
    report['cases'][path.stem]=dict(before_map_sha256=r['map_sha256'],unchanged=a==b,before_geometry_sha256=digest(a),after_geometry_sha256=digest(b),sweep_rectangles_m=rects)
    print(path.stem,'unchanged:',a==b)
(RAW/'access_regions_after_ramp_seam_repair.json').write_text(json.dumps(report,indent=2)+'\n')
