#!/usr/bin/env python3
"""Actual UE two-truck follow, stop and restart diagnostic with restoration."""
import hashlib
import json
import math
from pathlib import Path
import sys
import time
import airsim
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from logistics_park.common.messages import VehicleState
from logistics_park.safety.following_guard import following_speed_cap
from ue_dynamic_crossing_check import absolute_ue, make_pose, xyz, euclidean_xy
from ue_short_cargo_driving_check import audit_live
from ue_trailer_driving_check import ue_script, clear_nearby
ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'runs/ue_scene/two_vehicle_following'
NAMES = ('Car_0_0', 'Car_0_1')  # follower, leader; camera follows follower
SETTINGS = Path.home()/'Documents/AirSim/settings.json'


def controls(speed, target):
    if target < .08:
        return airsim.CarControls(brake=1)
    error = target-speed
    return airsim.CarControls(throttle=min(.45, max(0, .12+.3*error)),
                             brake=min(.4, max(0, -.3*error)))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    c = airsim.CarClient(timeout_value=5)
    if not c.ping() or c.simIsPause():
        raise RuntimeError('PIE must be running and unpaused')
    if any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in c.listVehicles()):
        raise RuntimeError('fleet must be stationary')
    scene = ue_script(c, 'export_scene.py', ROOT/'runs/ue_scene/scene.json')
    origin = [v/100 for v in scene['world_origin_cm']]
    survey = json.loads((ROOT/'config/topology/industrialarea_ue_survey.json').read_text())
    lane_x = survey['junctions']['j_2_0']['center_m'][0]-survey['lane_offset_m']
    home = {n:absolute_ue(c,n,origin)[0] for n in c.listVehicles()}
    api = {n:c.isApiControlEnabled(vehicle_name=n) for n in NAMES}
    digest = hashlib.sha256(SETTINGS.read_bytes()).hexdigest()
    report = {'status':'FAIL', 'vehicle_roles':{'follower':NAMES[0],'leader':NAMES[1]},
              'scope':'Ground Truth two-truck same-lane following, leader braking, restart; cargo conservative geometry audit',
              'parameters':{'vehicle_length_m':8.344,'standstill_gap_m':3,'time_headway_s':1.5,
                            'deceleration_m_s2':1,'reaction_time_s':.2,'target_rise_rate_m_s2':.5,'throttle_rise_rate_per_s':.12},'phases':[], 'failure':[], 'samples':[]}
    try:
        c.reset()
        report['temporarily_cleared'],_ = clear_nearby(c,origin)
        c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/prepare_dynamic_validation.py'))
        for n,y in zip(NAMES,(153,166)):
            c.enableApiControl(True,vehicle_name=n)
            oldlocal=xyz(c.simGetVehiclePose(vehicle_name=n)); oldworld=xyz(c.simGetObjectPose(n))
            target=[lane_x-origin[0],y-origin[1],origin[2]-.894]
            q=airsim.to_quaternion(0,0,math.pi/2)
            c.simPause(True)
            try:
                c.setCarControls(airsim.CarControls(brake=1),vehicle_name=n)
                c.simSetVehiclePose(make_pose([oldlocal[i]+target[i]-oldworld[i] for i in range(3)],
                    [q.x_val,q.y_val,q.z_val,q.w_val]),True,vehicle_name=n)
            finally:c.simPause(False)
        time.sleep(.5)
        for n in NAMES:
            (ROOT/'runs/ue_scene'/('following_'+n+'_history.jsonl')).unlink(missing_ok=True)
        c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/monitor_following.py'))
        c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/release_test_camera.py'))
        time.sleep(1)
        baseline={n:c.simGetCollisionInfo(vehicle_name=n).time_stamp for n in NAMES}
        print('camera ready: follower behind leader; starting in 3 seconds',flush=True)
        time.sleep(3)
        begin=time.monotonic()
        applied_targets={n:0.0 for n in NAMES}
        previous_throttle={n:0.0 for n in NAMES}
        last_control=begin
        for phase,duration in (('following',18),('leader_stop',18),('restart',18),('final_stop',12)):
            start=time.monotonic(); entries=[]; stopped_since=None;last_print=start
            print(phase+': starting',flush=True)
            while time.monotonic()-start<duration:
                now=time.monotonic(); poses={n:absolute_ue(c,n,origin)[0] for n in NAMES}
                speeds={n:abs(c.getCarState(vehicle_name=n).speed) for n in NAMES}
                audits={};errors=[]
                for n in NAMES:
                    a,e=audit_live(ROOT/'runs/ue_scene'/('following_'+n+'_live.json'));audits[n]=a;errors += [n+': '+v for v in e]
                    hit=c.simGetCollisionInfo(vehicle_name=n)
                    if hit.has_collided and hit.time_stamp!=baseline[n]:errors.append(n+' native collision: '+hit.object_name)
                    if abs(poses[n][0]-lane_x)>.35:errors.append(n+' lane deviation')
                    if poses[n][1]>253:errors.append('test corridor limit reached')
                def interval(a):
                    ends=[]
                    for b in a['bodies']:
                        angle=math.radians(b['yaw_deg']);h=b['half_size_m'];extent=abs(math.sin(angle))*h[0]+abs(math.cos(angle))*h[1]
                        ends += [b['centre_ue_m'][1]-extent,b['centre_ue_m'][1]+extent]
                    return min(ends),max(ends)
                physical_gap=interval(audits[NAMES[1]])[0]-interval(audits[NAMES[0]])[1]
                centre_gap=poses[NAMES[1]][1]-poses[NAMES[0]][1]
                if physical_gap<2.5:errors.append('body gap below 2.5 m')
                rear=VehicleState(NAMES[0],*poses[NAMES[0]][:2],math.pi/2,speed=speeds[NAMES[0]],edge_id='test_lane',s=poses[NAMES[0]][1])
                front=VehicleState(NAMES[1],*poses[NAMES[1]][:2],math.pi/2,speed=speeds[NAMES[1]],edge_id='test_lane',s=poses[NAMES[1]][1])
                cap=following_speed_cap(rear,front,8.344,standstill_gap=3,time_headway=1.5,deceleration=1,reaction_time=.2)
                stopping=phase in ('leader_stop','final_stop')
                leader_target=0 if stopping else 2
                desired_gap=3+1.5*speeds[NAMES[0]]
                target=max(0,min(2.5,cap,speeds[NAMES[1]]+.5*(centre_gap-8.344-desired_gap)))
                if speeds[NAMES[1]]<.05:target=0
                dt=min(.3,max(.01,now-last_control));last_control=now
                requested={NAMES[0]:target,NAMES[1]:leader_target}
                for n in NAMES:
                    applied_targets[n]=min(requested[n],applied_targets[n]+.5*dt)
                row={'t_s':now-begin,'phase_t_s':now-start,'phase':phase,'positions':poses,'speeds_m_s':speeds,
                     'body_gap_m':physical_gap,'conservative_gap_m':centre_gap-8.344,
                     'follower_speed_cap_m_s':cap,'follower_target_m_s':target,'body_audits':audits, 'leader_target_m_s':leader_target}
                entries.append(row);report['samples'].append(row)
                if errors:raise RuntimeError('; '.join(errors))
                for n in NAMES:
                    command=controls(speeds[n],applied_targets[n])
                    command.throttle=min(command.throttle,previous_throttle[n]+.12*dt)
                    previous_throttle[n]=command.throttle
                    c.setCarControls(command,vehicle_name=n)
                if now-last_print>4:
                    print(f'{phase}: front={speeds[NAMES[1]]:.2f}, rear={speeds[NAMES[0]]:.2f} m/s, body gap={physical_gap:.2f} m',flush=True);last_print=now
                if stopping and max(speeds.values())<.01:
                    stopped_since=stopped_since or now
                    if now-stopped_since>=2:break
                else:stopped_since=None
                time.sleep(.1)
            if stopping and (not stopped_since or time.monotonic()-stopped_since<2):raise RuntimeError(phase+' did not stop and hold')
            if not stopping and min(entries[-1]['speeds_m_s'].values())<1:raise RuntimeError(phase+' failed to establish motion')
            report['phases'].append({'phase':phase,'status':'PASS','duration_s':time.monotonic()-start,
                'minimum_body_gap_m':min(r['body_gap_m'] for r in entries),'final_speeds_m_s':entries[-1]['speeds_m_s'],
                'stop_time_s':{n:next((r['phase_t_s'] for r in entries if r['speeds_m_s'][n]<.01),None) for n in NAMES} if stopping else None,
                'hold_drift_m':{n:euclidean_xy(entries[-1]['positions'][n],next(r for r in entries if r['t_s']>=entries[-1]['t_s']-1.8)['positions'][n]) for n in NAMES} if stopping else None})
            print(phase+': PASS',flush=True)
        report['status']='PASS'
    except Exception as e:
        report['failure'].append(repr(e));print('FAIL',e,flush=True)
    finally:
        for n in NAMES:c.setCarControls(airsim.CarControls(brake=1),vehicle_name=n)
        c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/stop_following.py'))
        c.reset();time.sleep(.8)
        for n in NAMES:c.enableApiControl(api[n],vehicle_name=n)
        report['fleet_restored']=all(euclidean_xy(absolute_ue(c,n,origin)[0],p)<.1 for n,p in home.items())
        report['settings_unchanged']=hashlib.sha256(SETTINGS.read_bytes()).hexdigest()==digest
        if not report['fleet_restored'] or not report['settings_unchanged']:report['status']='FAIL'
        (OUT/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Two-vehicle check:',report['status'],flush=True)
    return 0 if report['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
