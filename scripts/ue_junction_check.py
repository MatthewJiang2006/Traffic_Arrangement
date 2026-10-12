#!/usr/bin/env python3
"""Exercise existing FCFS on actual UE two-truck junction movements."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import airsim
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from logistics_park.common.messages import VehicleState
from logistics_park.map.map_loader import load_map
from logistics_park.scheduling.road_capacity import RoadCapacity
from logistics_park.scheduling.fcfs import FCFSScheduler, EntranceRequest
from logistics_park.safety.following_guard import following_speed_cap
from ue_dynamic_crossing_check import absolute_ue, make_pose, xyz, euclidean_xy
from ue_short_cargo_driving_check import audit_live
from ue_following_check import controls
from ue_trailer_driving_check import ue_script,clear_nearby
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/ue_scene/two_vehicle_junction'
NAMES=('Car_0_0','Car_0_1')
JUNCTION='j_2_1'
SETTINGS=Path.home()/'Documents/AirSim/settings.json'


def support(a,axis):
    ends=[]
    for b in a['bodies']:
        angle=math.radians(b['yaw_deg']);h=b['half_size_m']
        extent=(abs(math.cos(angle))*h[0]+abs(math.sin(angle))*h[1]) if axis==0 else (abs(math.sin(angle))*h[0]+abs(math.cos(angle))*h[1])
        ends += [b['centre_ue_m'][axis]-extent,b['centre_ue_m'][axis]+extent]
    return min(ends),max(ends)


def teleport(c,name,point,yaw,origin):
    oldlocal=xyz(c.simGetVehiclePose(vehicle_name=name));oldworld=xyz(c.simGetObjectPose(name))
    target=[point[0]-origin[0],point[1]-origin[1],origin[2]-.894]
    q=airsim.to_quaternion(0,0,math.radians(yaw))
    c.enableApiControl(True,vehicle_name=name)
    was_paused=c.simIsPause()
    c.simPause(True)
    try:
        c.setCarControls(airsim.CarControls(brake=1),vehicle_name=name)
        c.simSetVehiclePose(make_pose([oldlocal[i]+target[i]-oldworld[i] for i in range(3)],
          [q.x_val,q.y_val,q.z_val,q.w_val]),True,vehicle_name=name)
    finally:c.simPause(was_paused)


def run_case(c,label,origin,graph,survey):
    cx,cy=survey['junctions'][JUNCTION]['center_m'];hx,hy=survey['junctions'][JUNCTION]['half_extent_m'];lane=survey['lane_offset_m']
    def north(y,goal):return {'axis':1,'fixed':cx-lane,'start':[cx-lane,y],'yaw':90,'entry':cy-hy,'exit':cy+hy,'goal':goal,'arm':'S','end':'N','incoming':'road_j_1_1_j_2_1','outgoing':'road_j_2_1_j_3_1'}
    def east(x,goal):return {'axis':0,'fixed':cy+lane,'start':[x,cy+lane],'yaw':0,'entry':cx-hx,'exit':cx+hx,'goal':goal,'arm':'W','end':'E','incoming':'road_j_2_0_j_2_1','outgoing':'road_j_2_1_j_2_2'}
    routes={NAMES[0]:north(198,239),NAMES[1]:east(-35,12)}
    if label=='same_lane_queue':routes={NAMES[0]:north(198,244),NAMES[1]:north(185,232)}
    if label=='blocked_exit':routes={NAMES[0]:north(198,239),NAMES[1]:north(231,250)}
    for r in routes.values():r['connector']=f"turn_{JUNCTION}_{r['arm']}_{r['end']}_single_single"
    result={'case':label,'status':'FAIL','samples':[],'failure':[],'routes':routes}
    c.reset();result['temporarily_cleared'],_=clear_nearby(c,origin)
    for n in NAMES:teleport(c,n,routes[n]['start'],routes[n]['yaw'],origin)
    time.sleep(.5)
    c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/monitor_following.py'))
    c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/release_test_camera.py'))
    time.sleep(.5)
    baseline={n:c.simGetCollisionInfo(vehicle_name=n).time_stamp for n in NAMES}
    capacity=RoadCapacity(graph);scheduler=FCFSScheduler((JUNCTION,),capacity)
    requested=set();entered=set();cleared=set();finished=set();release_at=None
    waiting_started={};waits={n:0 for n in NAMES}; max_simultaneous=0
    begin=time.monotonic();last_print=begin;last_tick=begin
    targets={n:0.0 for n in NAMES};throttle={n:0.0 for n in NAMES}
    print(label+': camera ready; starting in 3 seconds',flush=True);time.sleep(3)
    try:
        while time.monotonic()-begin<140:
            now=time.monotonic();t=now-begin;dt=min(.3,max(.01,now-last_tick));last_tick=now
            poses={n:absolute_ue(c,n,origin)[0] for n in NAMES};speeds={n:abs(c.getCarState(vehicle_name=n).speed) for n in NAMES}
            audits={};bounds={};errors=[]
            for n in NAMES:
                a,e=audit_live(ROOT/'runs/ue_scene'/('following_'+n+'_live.json'));audits[n]=a;errors += [n+': '+v for v in e]
                bounds[n]=support(a,routes[n]['axis'])
                hit=c.simGetCollisionInfo(vehicle_name=n)
                if hit.has_collided and hit.time_stamp!=baseline[n]:errors.append(n+' native collision '+hit.object_name)
                if abs(poses[n][1-routes[n]['axis']]-routes[n]['fixed'])>.35:errors.append(n+' lane deviation')
            active=[n for n in NAMES if not(label=='blocked_exit' and n==NAMES[1])]
            states={}
            for n in NAMES:
                r=routes[n];station=poses[n][r['axis']]
                edge=r['incoming'] if station<r['entry'] else r['connector'] if station<r['exit'] else r['outgoing']
                states[n]=VehicleState(n,*poses[n][:2],math.radians(r['yaw']),speed=speeds[n],edge_id=edge,s=max(0,station))
            capacity.update(states)
            for n in active:
                r=routes[n];low,high=bounds[n]
                if n not in requested and r['entry']-high<=2:
                    scheduler.request(EntranceRequest(n,JUNCTION,r['incoming'],r['connector'],r['outgoing'],t,max(0,poses[n][r['axis']])))
                    requested.add(n)
                if high>r['entry']+.05 and low<r['exit']:
                    if not scheduler.allows(n,JUNCTION,r['connector']):errors.append(n+' unauthorized entry')
                    elif n not in entered:scheduler.mark_entered(JUNCTION,n,t);entered.add(n)
                if n in entered and n not in cleared and low>r['exit']+.3:
                    scheduler.complete(JUNCTION,n,t);cleared.add(n)
            def exit_clear(edge,vehicle):
                r=routes[vehicle]
                return all(bounds[n][0]-r['exit']>=8.344+3 for n in NAMES
                           if n!=vehicle and routes[n]['outgoing']==edge and bounds[n][1]>r['exit'])
            scheduler.advance(t,can_enter=exit_clear)
            inside=[n for n in active if bounds[n][1]>routes[n]['entry']+.05 and bounds[n][0]<routes[n]['exit']]
            max_simultaneous=max(max_simultaneous,len(inside))
            if len(inside)>1:errors.append('junction simultaneously occupied')
            if capacity.violations:errors.append('road capacity exceeded')
            if label=='blocked_exit' and release_at is None and NAMES[0] in requested:
                release_at=t+6
                result['blocker_release_planned_s']=release_at
            for n in NAMES:
                r=routes[n];position=poses[n][r['axis']];low,high=bounds[n]
                allowed=n in cleared or scheduler.allows(n,JUNCTION,r['connector'])
                target=min(2,max(0,(r['goal']-position)*.6))
                if n==NAMES[1] and label=='blocked_exit':
                    target=target if release_at is not None and t>=release_at else 0
                elif not allowed:
                    distance=max(0,r['entry']-.8-high)
                    target=min(target,math.sqrt(2*.7*distance),distance/1.2)
                # Same-lane stopping cap also applies to a cleared/stationary
                # predecessor; do not stop just because a distant leader stopped.
                ahead=[m for m in NAMES if m!=n and routes[m]['axis']==r['axis'] and poses[m][r['axis']]>position]
                if ahead:
                    m=min(ahead,key=lambda v:poses[v][r['axis']]);rear=states[n];front=states[m]
                    rear.edge_id=front.edge_id='diagnostic_corridor'
                    target=min(target,following_speed_cap(rear,front,8.344,standstill_gap=3,time_headway=1.5,deceleration=1,reaction_time=.2))
                    if bounds[m][0]-high<2.5:errors.append('same-lane body gap below 2.5 m')
                safe_outlet_stop = label=='blocked_exit' and n==NAMES[0] and n in cleared and target<.08 and r['goal']-position<1
                if r['goal']-position<.25 or safe_outlet_stop:
                    target=0
                    if speeds[n]<.01:finished.add(n)
                if not allowed and n in active and n in requested and speeds[n]<.01:
                    waiting_started.setdefault(n,t)
                    waits[n]=max(waits[n],t-waiting_started[n])
                else:waiting_started.pop(n,None)
                targets[n]=min(target,targets[n]+.5*dt)
                command=controls(speeds[n],targets[n]);command.throttle=min(command.throttle,throttle[n]+.12*dt);throttle[n]=command.throttle
                c.setCarControls(command,vehicle_name=n)
            result['samples'].append({'t_s':t,'positions':poses,'speeds_m_s':speeds,'bounds_m':bounds,'owner':next(iter(scheduler.owners.values())).request.vehicle_id if scheduler.owners else None,'inside':inside,'body_audits':audits})
            if errors:raise RuntimeError('; '.join(errors))
            if now-last_print>5:
                print(f"{label}: owner={result['samples'][-1]['owner']}, entered={sorted(entered)}, cleared={sorted(cleared)}, speeds={[round(speeds[n],2) for n in NAMES]}",flush=True);last_print=now
            if set(active)<=cleared and len(finished)==2:break
            time.sleep(.1)
        else:raise RuntimeError('junction test timeout')
        order=[e['vehicle_id'] for e in scheduler.events if e['event']=='ENTERED']
        if label!='blocked_exit' and order!=list(NAMES):raise RuntimeError('unexpected entry order')
        waiting_vehicle=NAMES[0] if label=='blocked_exit' else NAMES[1]
        if waits[waiting_vehicle]<2:raise RuntimeError('insufficient stationary waiting evidence')
        if label=='blocked_exit':
            grant=next(e for e in scheduler.events if e['event']=='GRANTED')
            if grant['time']<release_at:raise RuntimeError('granted before outlet blocker moved')
            if scheduler.entry_blocked_attempts==0:raise RuntimeError('exit gate never blocked')
        result['status']='PASS'
    except Exception as e:result['failure'].append(repr(e));print(label,'FAIL',e,flush=True)
    finally:
        for n in NAMES:c.setCarControls(airsim.CarControls(brake=1),vehicle_name=n)
        c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/stop_following.py'))
        result.update(events=scheduler.events,maximum_simultaneous_junction_vehicles=max_simultaneous,stationary_wait_s=waits,entry_blocked_attempts=scheduler.entry_blocked_attempts,cleared=sorted(cleared))
        (OUT/(label+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        c.reset();time.sleep(.7)
    print(label,result['status'],flush=True)
    return result


def main():
    global OUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case',choices=('all','crossing_yield','same_lane_queue','blocked_exit'),default='all')
    parser.add_argument('--output',type=Path,default=OUT)
    args=parser.parse_args();OUT=args.output.resolve()
    labels=('crossing_yield','same_lane_queue','blocked_exit') if args.case=='all' else (args.case,)
    OUT.mkdir(parents=True,exist_ok=True);c=airsim.CarClient(timeout_value=5)
    if not c.ping() or c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in c.listVehicles()):raise RuntimeError('PIE fleet must be running and stationary')
    scene=ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json');origin=[v/100 for v in scene['world_origin_cm']]
    graph=load_map(ROOT/'config/topology/industrialarea_ue.json');survey=json.loads((ROOT/'config/topology/industrialarea_ue_survey.json').read_text())
    home={n:absolute_ue(c,n,origin)[0] for n in c.listVehicles()};api={n:c.isApiControlEnabled(vehicle_name=n) for n in NAMES};digest=hashlib.sha256(SETTINGS.read_bytes()).hexdigest()
    c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/prepare_dynamic_validation.py'))
    report={'cases':[],'controller':'existing exclusive FCFSScheduler and RoadCapacity; Ground Truth UE diagnostic'}
    try:
        for label in labels:report['cases'].append(run_case(c,label,origin,graph,survey))
    finally:
        c.reset();time.sleep(.7)
        for n in NAMES:c.enableApiControl(api[n],vehicle_name=n)
        report['fleet_restored']=all(euclidean_xy(absolute_ue(c,n,origin)[0],p)<.1 for n,p in home.items());report['settings_unchanged']=hashlib.sha256(SETTINGS.read_bytes()).hexdigest()==digest
        report['status']='PASS' if len(report['cases'])==len(labels) and all(r['status']=='PASS' for r in report['cases']) and report['fleet_restored'] and report['settings_unchanged'] else 'FAIL'
        (OUT/'acceptance.json').write_text(json.dumps({**report,'cases':[{k:v for k,v in r.items() if k!='samples'} for r in report['cases']]},indent=2)+'\n')
    print('UE junction acceptance',report['status'],flush=True)
    return 0 if report['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
