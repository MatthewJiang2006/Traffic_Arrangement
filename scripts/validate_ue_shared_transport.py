#!/usr/bin/env python3
"""Stage, launch and restore the formal two-truck shared-channel UE application."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
import airsim
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.ue_transport import configured_plans
from ue_trailer_driving_check import ue_script
from ue_dynamic_crossing_check import absolute_ue
from ue_junction_check import teleport
from validate_ue_remaining_service import clear_near_route
from validate_ue_loading_access import map_fingerprints
OUT=ROOT/'runs/ue_shared_transport'


def main():
    global OUT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',type=Path,default=ROOT/'config/runtime/industrialarea_ue_shared_transport.json')
    parser.add_argument('--output',type=Path,default=OUT)
    args=parser.parse_args();OUT=args.output
    OUT.mkdir(parents=True,exist_ok=True)
    c=airsim.CarClient(timeout_value=5)
    if not c.ping() or c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in c.listVehicles()):
        raise RuntimeError('PIE must be running with a stationary fleet')
    scene=ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json');origin=[v/100 for v in scene['world_origin_cm']]
    scenario=json.loads(args.scenario.read_text());scenario['adapter']['origin_ue_m']=origin
    if scenario.get('station_plan_sha256'):
        for field,path in (('station_plan_sha256',ROOT/'config/planning/ue_station_proposal.json'),
                           ('source_routes_sha256',ROOT/'config/planning/ue_remaining_service_routes.json')):
            if scenario[field]!=hashlib.sha256(path.read_bytes()).hexdigest():raise RuntimeError('Warehouse approved source changed: '+field)
        if scenario.get('source_map_sha256')!=map_fingerprints():raise RuntimeError('Warehouse access routes must match saved maps')
    original_map=Path(scenario['map'])
    frozen_map=OUT/'topology.json';frozen_map.write_bytes(original_map.read_bytes())
    scenario['original_map']=str(original_map)
    scenario['map']=str(frozen_map.resolve())
    plans=configured_plans(TransportMap(frozen_map),scenario)
    names=list(plans);home={n:absolute_ue(c,n,origin)[0][:2] for n in c.listVehicles()};apis={n:c.isApiControlEnabled(vehicle_name=n) for n in home}
    home_poses={n:c.simGetVehiclePose(vehicle_name=n) for n in home}
    settings=Path.home()/'Documents/AirSim/settings.json';digest=hashlib.sha256(settings.read_bytes()).hexdigest();maps=map_fingerprints()
    report=dict(status='FAIL',camera_changed=False,map_sha256=maps,scenario_map_sha256=hashlib.sha256(frozen_map.read_bytes()).hexdigest(),initial_fleet_xy_m=home,initial_fleet_poses={n:dict(position=[p.position.x_val,p.position.y_val,p.position.z_val],orientation=[p.orientation.x_val,p.orientation.y_val,p.orientation.z_val,p.orientation.w_val]) for n,p in home_poses.items()})
    cargo_props={}
    def console(script):
        if not c.simRunConsoleCommand(script):raise RuntimeError('UE Python command rejected')
    try:
        c.reset();console('py '+str(ROOT/'scripts/ue/prepare_dynamic_validation.py'))
        paths=[('clearance',[[x,y,0] for x,y in p.points],False) for p in plans.values()]
        report['temporarily_cleared']=clear_near_route(c,origin,paths)
        visibility_dir=ROOT/'runs/ue_scene/test_sites';visibility_dir.mkdir(parents=True,exist_ok=True)
        (visibility_dir/'idle_visibility_request.json').write_text(json.dumps(dict(active_vehicles=names)))
        report['idle_visibility']=ue_script(c,'hide_idle_test_fleet.py',visibility_dir/'idle_visibility_status.json')
        for n,p in plans.items():
            heading=p.heading(0);x,y=p.point(0);d=scenario['adapter'].get('wheelbase_m',2.3285)/2
            teleport(c,n,[x+d*math.cos(heading),y+d*math.sin(heading)],math.degrees(heading),origin)
            c.enableApiControl(False,vehicle_name=n)
            prefix=scenario.get('body_prefix','shared')+'_'+n;handle=scenario.get('body_prefix','shared')+'_monitor_'+n
            console("py import builtins,runpy; builtins.short_cargo_monitor_request="+repr(dict(vehicle=n,prefix=prefix,handle=handle))+"; runpy.run_path("+repr(str(ROOT/'scripts/ue/monitor_short_cargo.py'))+"); del builtins.short_cargo_monitor_request")
        if scenario.get('cargo_visuals'):
            for name in names:
                cargo_props[name]=c.simSpawnObject('TrafficCargoDrop_'+name,'SM_Truck_Trailer_1',
                    airsim.Pose(airsim.Vector3r(0,0,100)),airsim.Vector3r(1,1,1),False,False)
            events_path=OUT/'cargo_visual_commands.json';status_path=OUT/'cargo_visual_status.json'
            events_path.write_text(json.dumps({'commands':[]}))
            scenario['cargo_visuals'].update(events_path=str(events_path.resolve()),status_path=str(status_path.resolve()))
            request_path=ROOT/'runs/ue_scene/cargo_visual_request.json'
            request_path.write_text(json.dumps(dict(vehicles=names,prop_actors=cargo_props,**scenario['cargo_visuals'])))
            ue_script(c,'cargo_service_visuals.py',status_path)
        time.sleep(.5)
        path=OUT/'scenario.json';path.write_text(json.dumps(scenario,indent=2)+'\n')
        result=subprocess.run([str(ROOT/'.venv/bin/python'),'-m','logistics_park.runtime.ue_cli','--scenario',str(path),'--output',str(OUT/'dispatch'),'--body-directory',str(ROOT/'runs/ue_scene')],cwd=ROOT,timeout=scenario['max_runtime_s']+20)
        report['application_exit_code']=result.returncode
        dispatch=json.loads((OUT/'dispatch/acceptance.json').read_text())
        if result.returncode or dispatch['status']!='PASS':raise RuntimeError('formal shared transport failed: '+str(dispatch['failure']))
        report.update(status='PASS',dispatch_report=str((OUT/'dispatch/acceptance.json').relative_to(ROOT)) if OUT.is_absolute() else str(OUT/'dispatch/acceptance.json'),ue_physics_validated=True)
    except Exception as exc:
        report.update(failure=repr(exc),ue_physics_validated=False);print('UE shared fixture FAIL:',exc,flush=True)
    finally:
        visibility_dir=ROOT/'runs/ue_scene/test_sites'
        (visibility_dir/'idle_visibility_request.json').write_text(json.dumps(dict(action='RESTORE')))
        report['idle_visibility_restore']=ue_script(c,'hide_idle_test_fleet.py',visibility_dir/'idle_visibility_status.json')
        report['idle_visibility_restored']=(report['idle_visibility_restore']['status']=='RESTORED'
            and report['idle_visibility_restore']['actors']==report.get('idle_visibility',{}).get('actors',0))
        if scenario.get('cargo_visuals') and 'status_path' in scenario['cargo_visuals']:
            status_path=Path(scenario['cargo_visuals']['status_path'])
            if status_path.exists():
                report['cargo_visuals']=json.loads(status_path.read_text())
                visual=report['cargo_visuals'];events=visual.get('events',[])
                drops=[e for e in events if e['event']=='CARGO_DROPPED']
                attached=[e for e in events if e['event']=='CARGO_ATTACHED']
                removed=[e for e in events if e['event']=='GROUND_CARGO_REMOVED']
                expected=len(names)*scenario.get('cycles_per_vehicle',1)
                visual_pass=(visual['status']=='RUNNING' and not any(e['event']=='ERROR' for e in events)
                    and len(drops)==len(attached)==len(removed)==expected and not visual['ground_props']
                    and not any(visual['attached_visible'].values())
                    and all(5<=end['monotonic_s']-start['monotonic_s']<5.5 and start['id']==end['id']
                            and abs(start['bottom_z_m']-start['ground_z_m']-.01)<1e-6
                            for start,end in zip(drops,removed)))
                report['cargo_visuals_validated']=visual_pass
                if not visual_pass:report.update(status='FAIL',ue_physics_validated=False,failure='cargo appearance verification failed')
            (ROOT/'runs/ue_scene/cargo_visual_request.json').write_text(json.dumps(dict(action='STOP',status_path=str(OUT/'cargo_visual_cleanup.json'))))
            report['cargo_visual_cleanup']=ue_script(c,'cargo_service_visuals.py',OUT/'cargo_visual_cleanup.json')
        for prop in cargo_props.values():
            if not c.simDestroyObject(prop):report.update(status='FAIL',failure='cargo prop cleanup failed')
        for n in names:
            c.enableApiControl(True,vehicle_name=n);c.setCarControls(airsim.CarControls(brake=1),vehicle_name=n)
            console("py import builtins,unreal; h=getattr(builtins,"+repr(scenario.get('body_prefix','shared')+'_monitor_'+n)+",None); unreal.unregister_slate_post_tick_callback(h) if h is not None else None; setattr(builtins,"+repr(scenario.get('body_prefix','shared')+'_monitor_'+n)+",None)")
        c.reset();time.sleep(.5)
        c.simPause(True)
        try:
            for n,pose in home_poses.items():
                c.enableApiControl(True,vehicle_name=n)
                c.setCarControls(airsim.CarControls(brake=1),vehicle_name=n)
                c.simSetVehiclePose(pose,True,vehicle_name=n)
            stop_dir=ROOT/'runs/ue_scene/test_sites';stop_dir.mkdir(parents=True,exist_ok=True)
            (stop_dir/'motion_stop_request.json').write_text(json.dumps({'vehicles':list(home)}))
            stopped=ue_script(c,'stop_test_vehicle_motion.py',stop_dir/'motion_stop.json')
            if set(stopped['vehicles'])!=set(home):raise RuntimeError('incomplete physical fleet restoration')
        finally:c.simPause(False)
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            time.sleep(.2)
            if max(abs(c.getCarState(vehicle_name=n).speed) for n in home)<.02:break
        for n,enabled in apis.items():c.enableApiControl(enabled,vehicle_name=n)
        time.sleep(.3)
        report['fleet_restore_max_xy_error_m']=max(math.dist(p,absolute_ue(c,n,origin)[0][:2]) for n,p in home.items())
        report['fleet_max_speed_m_s']=max(abs(c.getCarState(vehicle_name=n).speed) for n in home)
        report['fleet_restored']=report['fleet_restore_max_xy_error_m']<.1 and report['fleet_max_speed_m_s']<.1
        report['api_restored']=all(c.isApiControlEnabled(vehicle_name=n)==enabled for n,enabled in apis.items())
        report['settings_unchanged']=hashlib.sha256(settings.read_bytes()).hexdigest()==digest
        report['saved_maps_unchanged']=map_fingerprints()==maps
        if not all(report[k] for k in ('fleet_restored','api_restored','settings_unchanged','saved_maps_unchanged','idle_visibility_restored')):report.update(status='FAIL',ue_physics_validated=False)
        ue_script(c,'place_warehouse_markers.py',ROOT/'runs/ue_scene/station_siting/marker_placement.json')
        (OUT/'acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Formal shared channel fixture:',report['status'],flush=True)
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
