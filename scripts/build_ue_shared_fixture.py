#!/usr/bin/env python3
"""Derive bounded two-way channel fixtures from measured, legal UE lanes."""
import argparse
import copy
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]


def build(junction=0):
    base=json.loads((ROOT/'config/topology/industrialarea_ue_transport.json').read_text())
    original={e['edge_id']:e for e in base['edges']};j=f'j_{junction}_1'
    east=f'road_j_{junction}_1_j_{junction}_2'
    west=f'road_j_{junction}_2_j_{junction}_1'
    keys=(f'road_j_{junction}_0_j_{junction}_1',f'turn_{j}_W_E_single_single',east+('__p0' if east+'__p0' in original else ''),west+('__p2' if west+'__p2' in original else ''),f'turn_{j}_E_W_single_single',f'road_j_{junction}_1_j_{junction}_0')
    edges=[copy.deepcopy(original[k]) for k in keys]
    ye=edges[1]['centerline'][0][1];yw=edges[4]['centerline'][0][1];boundary=edges[1]['centerline'][-1][0]
    goal=12 if junction==0 else 28;end=4 if junction==0 else 14
    edges[0].update(edge_id='shared_east_approach',from_node='shared_east_start',centerline=[[-35,ye],edges[0]['centerline'][-1]])
    edges[2].update(edge_id='shared_east_exit',to_node='shared_east_goal',centerline=[edges[2]['centerline'][0],[goal,ye]])
    edges[3].update(edge_id='shared_west_approach',from_node='shared_west_start',to_node=j+'_in_E',centerline=[[24,yw],edges[4]['centerline'][0]])
    edges[5].update(edge_id='shared_west_exit',to_node='shared_west_goal',centerline=[edges[5]['centerline'][0],[-35,yw]])
    nodes={}
    for e in edges:
        for key,point in ((e['from_node'],e['centerline'][0]),(e['to_node'],e['centerline'][-1])):
            nodes[key]=dict(node_id=key,node_type='WAYPOINT',position=point)
    movements=[]
    for route in (edges[:3],edges[3:]):
        for a,b in zip(route,route[1:]):
            movements.append(dict(movement_id=a['edge_id']+'__'+b['edge_id'],incoming_edge=a['edge_id'],outgoing_edge=b['edge_id'],maneuver='STRAIGHT',reference_path=b['centerline'],conflict_zones=[j],speed_limit=3))
    channel=f'shared_corridor_j{junction}'
    raw=dict(nodes=list(nodes.values()),edges=edges,movements=movements,sites={},control_zones={j:base['control_zones'][j]},physical_capacities=base['physical_capacities'],physical_centerlines=base['physical_centerlines'],speed_limits=base['speed_limits'],shared_resources={channel:dict(bounds=[boundary,yw-1.9,end,ye+1.9],edge_ids=[edges[2]['edge_id'],edges[3]['edge_id']],request_ahead_m=40,description='Measured east/west lanes, exclusive test channel; not warehouse routing')})
    suffix='' if junction==0 else f'_j{junction}'
    p=ROOT/f'config/topology/industrialarea_ue_shared_fixture{suffix}.json';p.write_text(json.dumps(raw,indent=2)+'\n')
    scenario=dict(application='shared_transport',map=str(p.relative_to(ROOT)),max_runtime_s=180,service_s=2,body_prefix='shared'+str(junction),expected_shared_order=['Car_0_0','Car_0_1'],minimum_wait_s=2,adapter=copy.deepcopy(json.loads((ROOT/'config/runtime/industrialarea_ue_four_truck.json').read_text())['adapter']),routes={'Car_0_0':dict(edges=[e['edge_id'] for e in edges[:3]],origin='east_probe_start',stops=['east_probe_goal']),'Car_0_1':dict(edges=[e['edge_id'] for e in edges[3:]],origin='west_probe_start',stops=['west_probe_goal'])})
    scenario['adapter']['vehicle_ids']=list(scenario['routes'])
    group='Q9Q10' if junction==1 else 'Q11Q12'
    scenario['verified_ground_actors']=['TrafficW1UnloadingRoadToCurb'] if junction==0 else ['TrafficWaiting'+group+'RoadToCurb','TrafficWaiting'+group+'CurbToHardstand']
    out=ROOT/f'config/runtime/industrialarea_ue_shared_transport{suffix}.json';out.write_text(json.dumps(scenario,indent=2)+'\n')
    return out


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--junction',type=int,choices=(0,1,3),default=0)
    print(build(parser.parse_args().junction))
