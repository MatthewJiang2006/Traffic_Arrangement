#!/usr/bin/env python3
"""Approved W5 loading -> W6 unloading -> legal road loop -> waiting berth."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from build_ue_turn_clearance import corrected_turn
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.routing.shortest_path import shortest_path

def build(vehicles=1,cycles=1):
    if vehicles not in (1,2) or cycles<1 or (vehicles==2 and cycles!=1):
        raise ValueError('single vehicle repeated cycles or two vehicles one cycle each')
    base=TransportMap(ROOT/'config/topology/industrialarea_ue_transport.json')
    plan=json.loads((ROOT/'config/planning/ue_station_proposal.json').read_text())
    access=json.loads((ROOT/'config/planning/ue_remaining_service_routes.json').read_text())
    wait=json.loads((ROOT/'config/planning/ue_waiting_access_routes.json').read_text())
    loop=list(shortest_path(base.graph,'j_2_1_in_S','j_1_1_out_N','astar','road_j_1_1_j_2_1').edges)
    base.graph.validate_route(tuple(['road_j_1_1_j_2_1',*loop]))
    stations={s['id']:s for s in plan['stations']};waiting={q['id']:q for q in plan['shared_waiting']}
    channel='W5_W6_access';edges=[];nodes={};movements=[];routes={};shared=[]
    physical=copy.deepcopy(base.raw['physical_centerlines']);capacities=copy.deepcopy(base.raw['physical_capacities'])
    d=2.3285/2
    def rear(path):return [[x-d*math.cos(math.radians(h)),y-d*math.sin(math.radians(h))] for x,y,h in path]
    for number in range(vehicles):
        name=f'Car_0_{number}';initial='Q2' if number==0 else 'Q1'
        destination=('Q1' if number==0 else 'Q2') if vehicles==2 else 'Q2'
        ids=[];cps=[];stops=[];current=None
        def add(label,points,alias,speed=2,reverse=False,kind='road',resource=None,maneuver='STRAIGHT',exclusive=False):
            nonlocal current
            key=name+'_'+label;frm=name+f'_node_{len(ids)}';to=name+f'_node_{len(ids)+1}'
            if current is not None:
                if math.dist(current,points[0])>1e-6:raise ValueError('path disconnected '+label)
            row=dict(edge_id=key,from_node=frm,to_node=to,centerline=points,width=3.8,speed_limit=speed,
                     storage_capacity=capacities.get(alias,1),kind=kind,resource=resource,capacity_alias=alias,maneuver=maneuver,reverse=reverse)
            if label.endswith('_to_loading'):row['shared_visit_start']=True
            edges.append(row);physical.setdefault(alias,points);capacities.setdefault(alias,1)
            for node,xy in ((frm,points[0]),(to,points[-1])):nodes[node]=dict(node_id=node,node_type='WAYPOINT',position=xy)
            if ids:movements.append(dict(movement_id=ids[-1]+'__'+key,incoming_edge=ids[-1],outgoing_edge=key,maneuver='STRAIGHT',reference_path=points,conflict_zones=[],speed_limit=speed))
            ids.append(key);current=points[-1]
            if exclusive:shared.append(key)
        def gear(direction):cps.append(dict(event='GEAR_CHANGE',after_edge_count=len(ids),direction=direction))
        def arrive(site,operation=None):
            cp=dict(leg=len(stops),event='ARRIVAL',site=site,node=name+f'_node_{len(ids)}',after_edge_count=len(ids),cycle=cycle)
            if operation:cp.update(operation=operation,channel_id=channel)
            cps.append(cp);stops.append(site)
        for cycle in range(cycles):
            q=initial if cycle==0 else destination
            load=[*stations['W5']['loading']['xy_m'],0]
            add(f'{cycle}_to_loading',rear([[*waiting[q]['xy_m'],0],load]),'w5_wait_lane',exclusive=True)
            arrive('W5-loading','LOADING')
            add(f'{cycle}_w5_exit',rear(access['sites']['W5-loading']['forward_exit']['path_xy_heading']),'w5_exit_lane',1.5,maneuver='RIGHT',exclusive=True)
            entry=access['sites']['W6-unloading']['reverse_entry']['path_xy_heading']
            add(f'{cycle}_w6_approach',[current,rear(entry)[0]],'road_j_1_1_j_2_1',3,exclusive=True)
            gear(-1)
            add(f'{cycle}_w6_reverse',rear(entry),'w6_reverse_lane',1,reverse=True,maneuver='LEFT',exclusive=True)
            arrive('W6-unloading','UNLOADING');gear(1)
            add(f'{cycle}_w6_exit',rear(access['sites']['W6-unloading']['forward_exit']['path_xy_heading']),'w6_exit_lane',1.5,maneuver='RIGHT',exclusive=True)
            add(f'{cycle}_north_road',[current,base.edges['road_j_1_1_j_2_1']['centerline'][-1]],'road_j_1_1_j_2_1',8)
            for i,key in enumerate(loop):
                e=base.edges[key]
                points=corrected_turn(e) if e.get('maneuver')=='RIGHT' and e['kind']=='junction' else e['centerline']
                speed=8 if e['kind']=='road' and e.get('maneuver','STRAIGHT')=='STRAIGHT' else e['speed_limit']
                add(f'{cycle}_loop_{i}',points,e['capacity_alias'],speed,kind=e['kind'],resource=e.get('resource'),maneuver=e.get('maneuver','STRAIGHT'),exclusive=key=='road_j_2_1_j_1_1')
                if e.get('maneuver')=='RIGHT':edges[-1].update(straight_until_m=math.dist(*points[:2]),maximum_path_error_m=1.2)
            back=wait['sites'][destination]['waiting_entry']['path_xy_heading']
            add(f'{cycle}_return_approach',[current,rear(back)[0]],'road_j_1_1_j_2_1',3,exclusive=True)
            gear(-1)
            add(f'{cycle}_return_waiting',rear(back),'w5_wait_lane',1,reverse=True,maneuver='LEFT',exclusive=True)
            arrive(destination)
            if cycle+1<cycles:gear(1)
        routes[name]=dict(edges=ids,origin=initial,stops=stops,checkpoints=cps)
    capacities['w5_wait_lane']=2
    physical['w5_wait_lane']=rear([[-64,173,0],[-40,173,0]])
    service={}
    for wid,role in [('W5','loading'),('W6','unloading')]:
        x,y=stations[wid][role]['xy_m'];service[wid+'-'+role]=dict(berth_nodes=[wid+'-'+role],bounds=[x-4.5,y-1.65,x+4.5,y+1.65],service_s=5)
    raw=dict(nodes=list(nodes.values()),edges=edges,movements=movements,sites={},control_zones=base.zones,
             physical_centerlines=physical,physical_capacities=capacities,speed_limits=base.raw['speed_limits'],service_stations=service,
             shared_resources={channel:dict(bounds=[-45,168,-6.7,196],edge_ids=shared,request_ahead_m=40,description='W5/W6 manoeuvres and opposite traffic through their short merge corridor reserved together; other roads remain concurrent')})
    suffix=f'{vehicles}v_{cycles}cycles';mp=ROOT/f'config/topology/industrialarea_ue_interwarehouse_{suffix}.json'
    raw['speed_limits']['straight_m_s']=8
    mp.write_text(json.dumps(raw,indent=2)+'\n')
    adapter=copy.deepcopy(json.loads((ROOT/'config/runtime/industrialarea_ue_w5_missions.json').read_text())['adapter'])
    adapter.update(vehicle_ids=list(routes),allow_reverse=True)
    adapter.update(max_throttle=.85,speed_rise_m_s2=1.2,throttle_rise_per_s=.4,speed_error_gain=1.)
    scenario=dict(application='shared_transport',interwarehouse=True,cycles_per_vehicle=cycles,map=str(mp.relative_to(ROOT)),scope='W5 loading -> W6 unloading -> legal road loop -> approved waiting berth',max_runtime_s=cycles*700+vehicles*180,
                  body_prefix='interwarehouse',minimum_wait_s=2 if vehicles==2 else 0,require_parallel_motion=vehicles==2,
                  service_s=3,adapter=adapter,verified_ground_actors=['TrafficW5RoadToCurb','TrafficW5CurbToHardstand','TrafficW6RoadToCurb','TrafficW6CurbToHardstand','TrafficW1LaneTaperLowY','TrafficW1LaneTaperHighY','TrafficW2LaneTaperLowY','TrafficW2LaneTaperHighY','Traffic_SeamBridge_Left','Traffic_SeamBridge_Middle','TrafficWaitingQ9Q10RoadToCurb','TrafficWaitingQ9Q10CurbToHardstand','TrafficWaitingQ11Q12RoadToCurb','TrafficWaitingQ11Q12CurbToHardstand'],routes=routes,
                  station_plan_sha256=hashlib.sha256((ROOT/'config/planning/ue_station_proposal.json').read_bytes()).hexdigest(),source_routes_sha256=hashlib.sha256((ROOT/'config/planning/ue_remaining_service_routes.json').read_bytes()).hexdigest(),source_map_sha256=access['map_sha256'],legal_road_loop_edges=loop)
    scenario['cargo_visuals']=dict(unloading_pads={'W6-unloading':dict(xy_m=[-36,190],yaw_deg=180)})
    sp=ROOT/f'config/runtime/industrialarea_ue_interwarehouse_{suffix}.json';sp.write_text(json.dumps(scenario,indent=2)+'\n')
    return sp

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--vehicles',type=int,default=1);p.add_argument('--cycles',type=int,default=1);a=p.parse_args();print(build(a.vehicles,a.cycles))
