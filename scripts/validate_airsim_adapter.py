#!/usr/bin/env python3
"""UE acceptance fixture only; traffic control runs in the project CLI/backend."""
import argparse
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path
import airsim
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig,AdapterFault
from logistics_park.common.messages import ControlCommand
from ue_junction_check import teleport
from ue_dynamic_crossing_check import absolute_ue,euclidean_xy
from ue_trailer_driving_check import ue_script,clear_nearby
OUT=ROOT/'runs/airsim_adapter_acceptance'
SETTINGS=Path.home()/'Documents/AirSim/settings.json'
NAMES=('Car_0_0','Car_0_1','Car_0_2','Car_1_0')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--dispatch-only',action='store_true');args=parser.parse_args()
    OUT.mkdir(parents=True,exist_ok=True);client=airsim.CarClient(timeout_value=5)
    if not client.ping() or client.simIsPause() or any(abs(client.getCarState(vehicle_name=n).speed)>.1 for n in client.listVehicles()):raise RuntimeError('PIE fleet must be stationary')
    scene=ue_script(client,'export_scene.py',ROOT/'runs/ue_scene/scene.json');origin=tuple(v/100 for v in scene['world_origin_cm'])
    scenario=json.loads((ROOT/'config/runtime/industrialarea_ue_four_truck.json').read_text());scenario['adapter']['origin_ue_m']=origin
    home={n:absolute_ue(client,n,origin)[0] for n in client.listVehicles()};apis={n:client.isApiControlEnabled(vehicle_name=n) for n in NAMES};digest=hashlib.sha256(SETTINGS.read_bytes()).hexdigest()
    report={'safety_cases':[],'status':'FAIL'}
    modes=('command_timeout','state_timeout','exit_braking')
    if args.dispatch_only:
        previous=json.loads((OUT/'acceptance.json').read_text());report['safety_cases']=previous['safety_cases']
        if len(report['safety_cases'])!=3 or any(r['status']!='PASS' for r in report['safety_cases']):raise RuntimeError('safety checks must pass before dispatch-only retry')
        (OUT/'dispatch_initial_failure.json').write_text(json.dumps(previous,indent=2)+'\n');modes=()
    try:
        client.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/prepare_dynamic_validation.py'))
        for mode in modes:
            client.reset();clear_nearby(client,origin);teleport(client,NAMES[0],scenario['routes'][NAMES[0]]['start'],90,origin);client.enableApiControl(False,vehicle_name=NAMES[0]);time.sleep(.3)
            adapter=AirSimAdapter(AirSimConfig((NAMES[0],),origin),OUT/(mode+'.jsonl'))
            adapter.open();sequence=0;begin=time.monotonic()
            try:
                while time.monotonic()-begin<3:
                    state=adapter.read_states()[NAMES[0]]
                    adapter.send(ControlCommand(NAMES[0],sequence,adapter.clock(),.8,0,ttl=.8));sequence+=1;time.sleep(.1)
                before=abs(adapter.read_states()[NAMES[0]].speed)
                if before<.1:raise RuntimeError(mode+' vehicle did not establish motion')
                if mode=='command_timeout':
                    deadline=time.monotonic()+1.5
                    while time.monotonic()<deadline:adapter.read_states();time.sleep(.1)
                    if not adapter.fault or 'command_timeout' not in adapter.fault:raise RuntimeError('watchdog did not expire command')
                elif mode=='state_timeout':
                    deadline=time.monotonic()+1.8
                    while time.monotonic()<deadline:
                        try:adapter.send(ControlCommand(NAMES[0],sequence,adapter.clock(),.8,0,ttl=.8));sequence+=1
                        except AdapterFault:break
                        time.sleep(.1)
                    if not adapter.fault or 'state_timeout' not in adapter.fault:raise RuntimeError('watchdog did not expire state')
                    time.sleep(.5)
                else:adapter.close()
                after=abs(client.getCarState(vehicle_name=NAMES[0]).speed)
                if after>=.01:raise RuntimeError(mode+' failed to stop')
                report['safety_cases'].append({'case':mode,'status':'PASS','moving_speed_m_s':before,'stopped_speed_m_s':after,'fault':adapter.fault})
                print(mode,'PASS',flush=True)
            finally:adapter.close();client.reset();time.sleep(.4)
        client.reset();clear_nearby(client,origin)
        for n in NAMES:
            teleport(client,n,scenario['routes'][n]['start'],90,origin)
            client.enableApiControl(False,vehicle_name=n)
        time.sleep(.5)
        client.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/monitor_four_trucks.py'));time.sleep(.5)
        scenario_path=OUT/'scenario.json';scenario_path.write_text(json.dumps(scenario,indent=2)+'\n')
        result=subprocess.run([str(ROOT/'.venv/bin/python'),'-m','logistics_park.runtime.ue_cli','--scenario',str(scenario_path),'--output',str(OUT/'four_trucks'),'--body-directory',str(ROOT/'runs/ue_scene')],cwd=ROOT)
        report['application_exit_code']=result.returncode
        if result.returncode!=0:raise RuntimeError('project UE dispatch failed')
        dispatch=json.loads((OUT/'four_trucks/acceptance.json').read_text());report['dispatch_status']=dispatch['status'];report['dispatch_report']='runs/airsim_adapter_acceptance/four_trucks/acceptance.json'
        report['status']='PASS'
    except Exception as e:report['failure']=repr(e);print('Adapter acceptance FAIL',e,flush=True)
    finally:
        client.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/stop_four_trucks.py'));client.reset();time.sleep(.8)
        for n in NAMES:client.enableApiControl(apis[n],vehicle_name=n)
        report['fleet_restored']=all(euclidean_xy(absolute_ue(client,n,origin)[0],p)<.1 for n,p in home.items())
        report['settings_unchanged']=hashlib.sha256(SETTINGS.read_bytes()).hexdigest()==digest
        if not report['fleet_restored'] or not report['settings_unchanged']:report['status']='FAIL'
        (OUT/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
        client.simRunConsoleCommand('py '+str(ROOT/'scripts/ue/release_test_camera.py'))
    print('Adapter acceptance',report['status'],flush=True)
    return 0 if report['status']=='PASS' else 1
if __name__=='__main__':raise SystemExit(main())
