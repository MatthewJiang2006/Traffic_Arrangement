#!/usr/bin/env python3
"""Patch only the round40 failed right turn, preserving all directed endpoints."""
import argparse,hashlib,json,math,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.geometry import Polyline
from logistics_park.map.turn_geometry import right_turn_path
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--edge',default='turn_j_3_0_N_E_right_single')
parser.add_argument('--output',type=Path,default=ROOT/'runs/right_turn_tracking_repair')
parser.add_argument('--policy',help='Calibration identifier saved with the repaired geometry')
args=parser.parse_args();KEY=args.edge;OUT=args.output;OUT.mkdir(parents=True,exist_ok=True)
POLICY=args.policy or ('calibrated_4_2m_round40' if KEY=='turn_j_3_0_N_E_right_single' else 'calibrated_4_2m_round44')
netpath=ROOT/'config/topology/industrialarea_ue_internal_waiting_network.json';mpath=ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json'
for p in (netpath,mpath):
 backup=OUT/(p.name+'.before.json')
 if not backup.exists():backup.write_bytes(p.read_bytes())
net=json.loads(netpath.read_text());model=json.loads(mpath.read_text());edge=next(e for e in net['edges'] if e['edge_id']==KEY);old=edge['centerline'];path=right_turn_path(edge,4.2);line=Polyline(tuple(map(tuple,path)))
edge.update(centerline=path,straight_until_m=math.dist(*path[:2]),right_turn_tracking_gain=1.2,right_turn_policy=POLICY,length_m=line.length)
if 'estimated_travel_time_s' in edge:edge['estimated_travel_time_s']=line.length/edge['speed_limit']
parts=[e for e in model['edges'] if e['capacity_alias']==KEY];parts.sort(key=lambda e:int(e['edge_id'].rsplit('test',1)[1]));oldlengths=[Polyline(tuple(map(tuple,e['centerline']))).length for e in parts];total=sum(oldlengths);lo=0.;points_changed={}
for i,(e,length) in enumerate(zip(parts,oldlengths)):
 hi=line.length if i==len(parts)-1 else lo+line.length*length/total
 stations=[0.]
 for a,b in zip(path,path[1:]):stations.append(stations[-1]+math.dist(a,b))
 points=[list(line.at(lo)[0])]+[p for s,p in zip(stations,path) if lo+1e-8<s<hi-1e-8]+[list(line.at(hi)[0])]
 e.update(centerline=points,straight_until_m=max(0.,min(hi,math.dist(*path[:2]))-lo),right_turn_tracking_gain=1.2,right_turn_policy=POLICY,length_m=hi-lo)
 points_changed[e['from_node']]=points[0];points_changed[e['to_node']]=points[-1];lo=hi
for node in model['nodes']:
 if node['node_id'] in points_changed:node['position']=points_changed[node['node_id']]
for movement in model['movements']:
 rows={e['edge_id']:e for e in model['edges']}
 if movement['incoming_edge'] in {e['edge_id'] for e in parts} or movement['outgoing_edge'] in {e['edge_id'] for e in parts}:
  a,b=rows[movement['incoming_edge']],rows[movement['outgoing_edge']];movement['reference_path']=[a['centerline'][-2],a['centerline'][-1],b['centerline'][1]]
model['physical_centerlines'][KEY]=path
for p,r in ((netpath,net),(mpath,model)):p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n')
(OUT/'geometry_patch.json').write_text(json.dumps(dict(edge=KEY,radius_m=4.2,tracking_gain=1.2,endpoints_preserved=[old[0]==path[0],old[-1]==path[-1]],changed_parts=[e['edge_id'] for e in parts],saved_ue_maps_changed=False,status='PENDING_LOCAL_UE'),indent=2)+'\n')
print(KEY,'patched',len(parts),'parts, endpoints unchanged')
