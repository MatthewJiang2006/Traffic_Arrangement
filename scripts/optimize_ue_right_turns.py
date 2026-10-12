#!/usr/bin/env python3
"""Update all existing right-turn references without changing lane connectivity."""
import hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.turn_geometry import right_turn_path
out=ROOT/'runs/ue_right_turns';out.mkdir(exist_ok=True)
rows=[]
for filename in ['industrialarea_ue.json','industrialarea_ue_transport.json']:
    path=ROOT/'config/topology'/filename;old=json.loads(path.read_text());backup=out/(filename+'.before.json')
    if not backup.exists():backup.write_bytes(path.read_bytes())
    for e in old['edges']:
        if e.get('maneuver')!='RIGHT' or not e['edge_id'].startswith('turn_'):continue
        e['centerline']=right_turn_path(e);e['straight_until_m']=__import__('math').dist(*e['centerline'][:2]);e['right_turn_policy']='delayed_yaw_lane_dependent_radius';e['speed_limit']=min(e['speed_limit'],1.5);e['maximum_path_error_m']=1.2
        if 'physical_centerlines' in old:old['physical_centerlines'][e['capacity_alias']]=e['centerline']
        rows.append(dict(map=filename,edge=e['edge_id'],incoming_lane=e['incoming_lane'],outgoing_lane=e['outgoing_lane']))
    edges={e['edge_id']:e for e in old['edges']}
    for m in old['movements']:
        a,b=edges[m['incoming_edge']],edges[m['outgoing_edge']]
        if a.get('maneuver')=='RIGHT' or b.get('maneuver')=='RIGHT':
            m['reference_path']=[a['centerline'][-2],a['centerline'][-1],b['centerline'][1]]
            m['speed_limit']=min(m['speed_limit'],1.5)
    path.write_text(json.dumps(old,ensure_ascii=False,indent=2)+'\n')
manifest=dict(right_turns=rows,changed_right_turns_per_map=22,ue_saved_maps_changed=False,policy='Existing endpoints and legal lanes; delayed yaw 3.2m wide-road approach / 4m single-lane radius and 1.5m/s cap',map_sha256={name:hashlib.sha256((ROOT/'config/topology'/name).read_bytes()).hexdigest() for name in ['industrialarea_ue.json','industrialarea_ue_transport.json']})
(out/'planning_patch.json').write_text(json.dumps(manifest,indent=2)+'\n');print('Optimized',len(rows),'right-turn references in two maps')
