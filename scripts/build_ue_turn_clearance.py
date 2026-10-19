#!/usr/bin/env python3
"""Delay right-turn yaw to avoid cutting the inner curb with truck rear wheels."""
import argparse,copy,json,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

import sys
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.map.turn_geometry import right_turn_path as corrected_turn,cardinal_axes

def axes(edge):return cardinal_axes(edge['centerline'])

def build(key):
    b=json.loads((ROOT/'config/topology/industrialarea_ue_transport.json').read_text());turn=next(e for e in b['edges'] if e['edge_id']==key)
    t=copy.deepcopy(turn);t['centerline']=corrected_turn(t);t['straight_until_m']=math.dist(*t['centerline'][:2]);incoming,outgoing=axes(turn)
    before=next(e for e in b['edges'] if e['to_node']==turn['from_node'] and e['kind']=='road')
    after=next(e for e in b['edges'] if e['from_node']==turn['to_node'] and e['kind']=='road')
    a,z=t['centerline'][0],t['centerline'][-1]
    rows=[dict(edge_id='approach',from_node='a',to_node=t['from_node'],centerline=[[a[i]-15*incoming[i] for i in (0,1)],a],width=3.8,speed_limit=3,storage_capacity=2,kind='road',resource=None,capacity_alias=before['capacity_alias']),t,
          dict(edge_id='exit',from_node=t['to_node'],to_node='b',centerline=[z,[z[i]+12*outgoing[i] for i in (0,1)]],width=3.8,speed_limit=4,storage_capacity=2,kind='road',resource=None,capacity_alias=after['capacity_alias'])]
    nodes={n:dict(node_id=n,node_type='WAYPOINT',position=p) for e in rows for n,p in [(e['from_node'],e['centerline'][0]),(e['to_node'],e['centerline'][-1])]}
    movements=[dict(movement_id=a['edge_id']+'__'+z['edge_id'],incoming_edge=a['edge_id'],outgoing_edge=z['edge_id'],maneuver='STRAIGHT',reference_path=z['centerline'],conflict_zones=[],speed_limit=z['speed_limit']) for a,z in zip(rows,rows[1:])]
    b.update(nodes=list(nodes.values()),edges=rows,movements=movements,sites={},shared_resources={})
    suffix='' if key=='turn_j_2_2_W_N_single_single' else '_'+key
    mp=ROOT/f'config/topology/industrialarea_ue_turn_clearance{suffix}.json';mp.write_text(json.dumps(b,indent=2)+'\n')
    s=json.loads((ROOT/'config/runtime/industrialarea_ue_parallel_transport.json').read_text());s.update(map=str(mp.relative_to(ROOT)),scope=key+' delayed right-turn yaw inside surveyed drivable surface',body_prefix='turnclear',max_runtime_s=140,require_parallel_motion=False,minimum_wait_s=0,routes={'Car_0_0':dict(edges=[e['edge_id'] for e in rows],origin='turn_approach',stops=['turn_exit'])});s['adapter']['vehicle_ids']=['Car_0_0']
    scene=json.loads((ROOT/'runs/ue_scene/scene.json').read_text())
    s['verified_ground_actors']=sorted({a['name'] for a in scene['actors'] if a['name'].startswith('Traffic') and ('Ramp' in a['name'] or 'ToCurb' in a['name'] or 'ToHardstand' in a['name'] or 'LaneTaper' in a['name'] or 'SeamBridge' in a['name'])})
    sp=ROOT/f'config/runtime/industrialarea_ue_turn_clearance{suffix}.json';sp.write_text(json.dumps(s,indent=2)+'\n');return sp

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--edge',default='turn_j_2_2_W_N_single_single');a=p.parse_args();print(build(a.edge))
