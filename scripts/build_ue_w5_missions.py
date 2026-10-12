#!/usr/bin/env python3
"""Bind approved W5 berths and proven Q2-before-Q1 forward routes to formal control."""
import hashlib
import json
import math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
plan=json.loads((ROOT/'config/planning/ue_station_proposal.json').read_text())
source=json.loads((ROOT/'config/planning/ue_remaining_service_routes.json').read_text())
w5=next(s for s in plan['stations'] if s['id']=='W5')
waiting={p['id']:p for p in plan['shared_waiting']}
base=json.loads((ROOT/'config/topology/industrialarea_ue_transport.json').read_text())
exit_pawn=source['sites']['W5-unloading']['forward_exit']['path_xy_heading']
assert exit_pawn[0][:2]==w5['unloading']['xy_m']
d=2.3285/2

def rear(path):
    return [[x-d*math.cos(math.radians(yaw)),y-d*math.sin(math.radians(yaw))] for x,y,yaw in path]

edges=[];routes={};nodes={};movements=[]
channel='W5_work_access';north='road_j_1_1_j_2_1'
physical={};capacities={north:base['physical_capacities'][north]}
for n,q,goal in (('Car_0_0','Q2',203),('Car_0_1','Q1',189)):
    start=[*waiting[q]['xy_m'],waiting[q]['heading_deg']];load=[*w5['loading']['xy_m'],0];unload=[*w5['unloading']['xy_m'],0]
    stages=[('waiting_to_loading',[start,load],'w5_wait_lane',2),('loading_to_unloading',[load,unload],'w5_work_lane',1),('forward_exit',exit_pawn,'w5_exit_lane',1),('north_recovery',[exit_pawn[-1],[-12.104,goal,90]],north,capacities[north])]
    ids=[]
    for i,(label,path,alias,capacity) in enumerate(stages):
        points=rear(path);key=n+'_'+label;frm=n+f'_node_{i}';to=n+f'_node_{i+1}'
        speed=4 if label=='north_recovery' else 1.5 if label=='forward_exit' else 2
        row=dict(edge_id=key,from_node=frm,to_node=to,centerline=points,width=3.8,speed_limit=speed,storage_capacity=capacity,kind='road',resource=None,capacity_alias=alias,maneuver='RIGHT' if label=='forward_exit' else 'STRAIGHT')
        edges.append(row);ids.append(key);capacities[alias]=capacity
        physical.setdefault(alias,points)
        for node,xy in ((frm,points[0]),(to,points[-1])):nodes[node]=dict(node_id=node,node_type='WAYPOINT',position=xy)
        if i:movements.append(dict(movement_id=ids[-2]+'__'+key,incoming_edge=ids[-2],outgoing_edge=key,maneuver='STRAIGHT',reference_path=points,conflict_zones=[],speed_limit=speed))
    checkpoints=[dict(leg=i,site='W5-'+role,node=n+f'_node_{i+1}',after_edge_count=i+1,event='ARRIVAL',operation=role.upper(),channel_id=channel) for i,role in enumerate(('loading','unloading'))]
    routes[n]=dict(edges=ids,origin=q,stops=['W5-loading','W5-unloading'],checkpoints=checkpoints)
# Common wait-lane station keeps FIFO/following comparisons in a shared coordinate frame.
physical['w5_wait_lane']=rear([[-64,173,0],[-40,173,0]])
services={}
for role in ('loading','unloading'):
    x,y=w5[role]['xy_m'];services['W5-'+role]=dict(berth_nodes=['W5-'+role],bounds=[x-4.5,y-1.65,x+4.5,y+1.65],service_s=5)
raw=dict(nodes=list(nodes.values()),edges=edges,movements=movements,sites={},control_zones={},physical_centerlines=physical,physical_capacities=capacities,speed_limits=base['speed_limits'],service_stations=services,shared_resources={channel:dict(bounds=[-45,168,-6,184],edge_ids=[e['edge_id'] for e in edges if not e['edge_id'].endswith('north_recovery')],request_ahead_m=40,description='Only W5 work/exit area; Q1/Q2 waiting outside bounds')})
map_path=ROOT/'config/topology/industrialarea_ue_w5_missions.json';map_path.write_text(json.dumps(raw,indent=2)+'\n')
scenario=dict(application='shared_transport',map=str(map_path.relative_to(ROOT)),scope='W5 local waiting/loading/unloading/road exit; not inter-warehouse transport',max_runtime_s=320,body_prefix='warehouse',expected_shared_order=list(routes),minimum_wait_s=2,require_parallel_motion=True,adapter=json.loads((ROOT/'config/runtime/industrialarea_ue_four_truck.json').read_text())['adapter'],verified_ground_actors=['TrafficW5RoadToCurb','TrafficW5CurbToHardstand','TrafficW6RoadToCurb','TrafficW6CurbToHardstand'],routes=routes,station_plan_sha256=hashlib.sha256((ROOT/'config/planning/ue_station_proposal.json').read_bytes()).hexdigest(),source_routes_sha256=hashlib.sha256((ROOT/'config/planning/ue_remaining_service_routes.json').read_bytes()).hexdigest(),source_map_sha256=source['map_sha256'])
scenario['adapter']['vehicle_ids']=list(routes)
(ROOT/'config/runtime/industrialarea_ue_w5_missions.json').write_text(json.dumps(scenario,indent=2)+'\n')
print('W5 two local missions; Q2 before Q1; each loading/unloading service 5 seconds')
