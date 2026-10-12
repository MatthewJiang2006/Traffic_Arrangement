#!/usr/bin/env python3
"""Measure faster straight travel and stopping on the surveyed right lane."""
import json
import math
import sys
import time
from pathlib import Path
import airsim

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from logistics_park.adapters.airsim import AirSimAdapter, AirSimConfig
from logistics_park.common.messages import ControlCommand
from logistics_park.map.map_loader import load_map
from logistics_park.runtime.body_observer import FileBodyObserver
from logistics_park.runtime.speed_limits import SpeedLimits
from ue_dynamic_crossing_check import absolute_ue
from ue_junction_check import teleport
from ue_trailer_driving_check import ue_script
from validate_ue_test_sites import zero_motion

NAME = 'Car_0_0'
OUT = ROOT/'runs/ue_scene/speed_increase/straight'


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    c = airsim.CarClient(timeout_value=5)
    names = c.listVehicles()
    if c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed) > .01 for n in names):
        raise RuntimeError('stationary, unpaused PIE required')
    origin = tuple(v/100 for v in ue_script(c, 'export_scene.py', ROOT/'runs/ue_scene/scene.json')['world_origin_cm'])
    homes = {n:absolute_ue(c,n,origin)[0][:2] for n in names}
    home_pose = c.simGetObjectPose(NAME)
    home_yaw = math.degrees(airsim.to_eularian_angles(home_pose.orientation)[2])
    previous_api = c.isApiControlEnabled(vehicle_name=NAME)
    limits = SpeedLimits(**json.loads((ROOT/'config/runtime/ue_speed_limits.json').read_text()))
    edge = load_map(ROOT/'config/topology/industrialarea_ue.json').edges['road_j_1_0_j_2_0_right']
    x = edge.centerline.at(10)[0][0]
    start_y, goal_y = 154., 198.
    result = {'status':'FAIL','failure':[], 'speed_limits':vars(limits),
              'lane_edge':edge.edge_id, 'samples':[], 'camera_changed':False}
    adapter = AirSimAdapter(AirSimConfig((NAME,),origin,throttle_rise_per_s=.24,
                                         speed_rise_m_s2=.8,max_throttle=.7,
                                         speed_error_gain=.6), OUT/'adapter.jsonl')
    try:
        zero_motion(c,(NAME,))
        teleport(c,NAME,[x,start_y],90,origin)
        c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/monitor_following.py'))
        time.sleep(.5)
        observer = FileBodyObserver(ROOT/'runs/ue_scene',prefix='following')
        adapter.open()
        baseline = adapter.telemetry[NAME]['collision']['time_stamp']
        begin = time.monotonic()
        for sequence in range(1400):
            state = adapter.read_states()[NAME]
            audit, errors = observer.read(NAME)
            hit = adapter.telemetry[NAME]['collision']
            if hit['has_collided'] and hit['time_stamp'] != baseline:
                errors.append('native collision '+hit['object_name'])
            if abs(state.x-x) > .5 or state.y > goal_y+.5:
                errors.append('lane deviation or stopping overshoot')
            if errors:
                raise RuntimeError('; '.join(errors))
            remaining = max(0.,goal_y-state.y)
            target = min(limits.straight_m_s,limits.approach(0.,remaining),remaining*.8)
            if remaining < .22:
                target = 0.
            aim_heading = math.atan2(5.,x-state.x)
            alpha = (aim_heading-state.heading+math.pi)%(2*math.pi)-math.pi
            delta = math.atan2(2*adapter.config.wheelbase_m*math.sin(alpha),5.)
            omega = target*math.tan(delta)/adapter.config.wheelbase_m if target > .05 else 0.
            adapter.send(ControlCommand(NAME,sequence,adapter.clock(),target,omega,ttl=.8))
            result['samples'].append({'t_s':time.monotonic()-begin,'xy':[state.x,state.y],
                                      'speed_m_s':state.speed,'target_m_s':target})
            if target == 0. and abs(state.speed) < .03:
                break
            time.sleep(.1)
        else:
            raise RuntimeError('straight speed validation timeout')
        result['maximum_actual_speed_m_s'] = max(abs(s['speed_m_s']) for s in result['samples'])
        result['stopping_error_m'] = abs(state.y-goal_y)
        result['maximum_lateral_error_m'] = max(abs(s['xy'][0]-x) for s in result['samples'])
        if result['maximum_actual_speed_m_s'] < 1.5:
            raise RuntimeError('actual speed did not increase enough')
        result['status'] = 'PASS'
    except BaseException as exc:
        result['failure'].append(repr(exc))
        adapter.fail_stop('straight speed check: '+str(exc))
        if isinstance(exc,(KeyboardInterrupt,SystemExit)):
            raise
    finally:
        adapter.close()
        c.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/stop_following.py'))
        zero_motion(c,(NAME,))
        c.simPause(True)
        try:
            teleport(c,NAME,homes[NAME],home_yaw,origin)
            zero_motion(c,(NAME,))
        finally:
            c.simPause(False)
        time.sleep(.5)
        c.enableApiControl(previous_api,vehicle_name=NAME)
        result['close_result'] = adapter.close_result
        result['fleet_restored'] = all(math.dist(absolute_ue(c,n,origin)[0][:2],homes[n]) < .1 for n in names)
        if not result['fleet_restored'] or adapter.fault or not adapter.close_result.get('api_restored'):
            result['status'] = 'FAIL'
        (OUT/'acceptance.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Straight speed check:',result['status'],result['failure'],flush=True)
    return 0 if result['status']=='PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
