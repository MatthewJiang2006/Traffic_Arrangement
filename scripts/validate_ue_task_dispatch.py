#!/usr/bin/env python3
"""Two simultaneous physical transport tasks driven by TaskDispatcher.

Initial test staging is reversible; task execution uses throttle/steering only.
Waiting routes and dynamic gates are validated separately in the light runner.
"""
import json,math,time,traceback,hashlib
from pathlib import Path
from validate_ue_node_pairs import (ROOT,airsim,rear,AirSimAdapter,AirSimConfig,
    ControlCommand,VehicleState,FileBodyObserver,TransportMap,TransportController,
    envelope,observed_boxes,supported_ramp_contact,ue_script,absolute_ue,map_fingerprints)
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork,WorkStatus
from logistics_park.runtime.task_execution import ValidatedServiceRoutes
from logistics_park.safety.collision import overlaps

OUT=ROOT/'runs/task_dispatch';OUT.mkdir(parents=True,exist_ok=True)
FIXTURE=ROOT/'runs/ue_node_pairs'

def main():
    c=airsim.CarClient(timeout_value=10);names=c.listVehicles()
    if c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in names):
        raise RuntimeError('Stationary unpaused fleet required')
    scene=ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json')
    if {v['name'] for v in scene['pie_vehicles']}!=set(names):raise RuntimeError('Actor fleet disagrees; restart PIE')
    ue_script(c,'prepare_dynamic_validation.py',ROOT/'runs/ue_scene/dynamic_editor_settings.json')
    origin=tuple(v/100 for v in scene['world_origin_cm'])
    home={n:c.simGetVehiclePose(vehicle_name=n) for n in names}
    home_ue={n:absolute_ue(c,n,origin)[0] for n in names}
    apis={n:c.isApiControlEnabled(vehicle_name=n) for n in names};maps=map_fingerprints()
    specpath=ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json'
    spec=json.loads(specpath.read_text());model=TransportMap(ROOT/spec['map'])
    network=ServiceNetwork(ROOT/'config/topology/industrialarea_ue_service_network.json')
    routes=ValidatedServiceRoutes(network,model,spec['jobs'])
    fleet=FleetRegistry();dispatch=TaskDispatcher(network,fleet,service_s=5.)
    tasks=[('Car_0_0','W1-loading','W3-unloading'),('Car_0_1','W2-loading','W4-unloading')]
    pool=[t[0] for t in tasks];plans={};initial={};visits={};moves={};sources={}
    # Check every geometry binding before modifying the scene.
    from logistics_park.scheduling.task_dispatch import DispatchMove
    for name,a,b in tasks:
        initial[name]=routes.plan(DispatchMove(0,name,name,'precheck',a,b,network.route(a,b),False))
    controller=TransportController(model,plans,service_s=0,shared_resources=True,collect_at_destination=True)
    cfg=json.loads((ROOT/'config/runtime/industrialarea_ue_interwarehouse_1v_1cycles.json').read_text())['adapter']
    cfg.update(vehicle_ids=tuple(pool),origin_ue_m=origin,command_ttl_s=2,state_timeout_s=5.)
    observer=FileBodyObserver(ROOT/'runs/ue_scene',prefix='task',wheel_diagnostic_only=True)
    ground={a['name'] for a in scene['actors'] if a['name'].startswith('Traffic') and any(t in a['name'] for t in ('Ramp','ToCurb','ToHardstand','LaneTaper','SeamBridge'))}
    adapter=None;baseline={};trajectory=None
    report=dict(status='RUNNING',scope='Two physical loading-to-unloading tasks; 5-second services; no waiting/gate physical claim',
        expected_tasks=2,completed_tasks=0,maximum_moving_vehicles=0,body_obstacle_contacts=0,vehicle_collisions=0,
        camera_changed=False,map_sha256=maps,source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (specpath,ROOT/spec['map'],ROOT/'config/topology/industrialarea_ue_service_network.json')},
        requested_world_time_scale=4,ue_physics_validated=False,cargo_visual_effect_validated=False)
    def console(command):
        if not c.simRunConsoleCommand(command):raise RuntimeError('UE rejected command')
    def fixture(action,vehicles=None):
        (FIXTURE/'fleet_request.json').write_text(json.dumps(dict(action=action,vehicles=vehicles or names,time_scale=4)))
        return ue_script(c,'node_pair_fleet_fixture.py',FIXTURE/'fleet_status.json')
    def stop_monitor(name):
        console("py import builtins,unreal; h=getattr(builtins,"+repr('task_monitor_'+name)+",None); unreal.unregister_slate_post_tick_callback(h) if h is not None else None; setattr(builtins,"+repr('task_monitor_'+name)+",None)")
    try:
        fixture('INIT');adapter=AirSimAdapter(AirSimConfig(**cfg),OUT/'physical_adapter.jsonl');adapter.open()
        for name,a,b in tasks:
            p=initial[name];x,y=p.point(0);h=p.heading(0)
            target=[x+1.16425*math.cos(h),y+1.16425*math.sin(h),.894]
            hp=home[name].position;world=home_ue[name]
            pose=airsim.Pose(airsim.Vector3r(hp.x_val+target[0]-world[0],hp.y_val+target[1]-world[1],hp.z_val-target[2]+world[2]),airsim.to_quaternion(0,0,h))
            c.simPause(True)
            try:
                fixture('ACTIVATE',[name]);c.simSetVehiclePose(pose,True,vehicle_name=name)
            finally:c.simPause(False)
            prefix='task_'+name;live=ROOT/'runs/ue_scene'/f'{prefix}_live.json';live.unlink(missing_ok=True)
            console("py import builtins,runpy; builtins.short_cargo_monitor_request="+repr(dict(vehicle=name,prefix=prefix,handle='task_monitor_'+name,exclude_inactive_fixture=True))+"; runpy.run_path("+repr(str(ROOT/'scripts/ue/monitor_short_cargo.py'))+"); del builtins.short_cargo_monitor_request")
            deadline=time.monotonic()+2
            while not live.exists() and time.monotonic()<deadline:time.sleep(.02)
            if not live.exists() or math.dist(json.loads(live.read_text())['truck_ue_m'][:2],target[:2])>.3:
                raise RuntimeError('Invalid staged start '+name)
            v=fleet.adopt_present_vehicle(name,0,fully_inside=True,resources=[]);visits[name]=v.visit_id
            dispatch.place_vehicle(name,v.visit_id,a,0,stopped=True)
            dispatch.submit(TransportWork('haul-'+name,a,b))
            baseline[name]=c.simGetCollisionInfo(vehicle_name=name).time_stamp
            sources[name]=envelope(VehicleState(name,x,y,h))
        begin=time.monotonic();last_status=0;sequence=1
        trajectory=(OUT/'physical_trajectory.jsonl').open('w')
        while True:
            now=time.monotonic()-begin
            if now>420:raise RuntimeError('Seven-minute targeted task test limit')
            measured=adapter.read_states();states={n:rear(measured[n]) for n in pool};boxes={};audits={}
            for name,state in states.items():
                audit,errors=observer.read(name);audits[name]=audit;boxes[name]=observed_boxes(audit)
                if errors:
                    report['body_obstacle_contacts']+=sum(bool(b['static_sweep_hit']) for b in audit['bodies'])
                    report['vehicle_collisions']+=sum(bool(b['parked_vehicle_overlap']) for b in audit['bodies'])
                    raise RuntimeError(name+' body audit '+str(errors))
                hit=adapter.telemetry[name]['collision']
                if hit['has_collided'] and hit['time_stamp']!=baseline[name]:
                    row=dict(collision=hit['object_name'],collision_normal=[hit['normal'][k] for k in ('x_val','y_val','z_val')],
                        collision_penetration_m=hit['penetration_depth'],wheel_clearances_m=[w['clearance_m'] for w in audit['wheels']],
                        wheel_surfaces=[w['road_actor'] for w in audit['wheels']],body_hits=[b['static_sweep_hit'] for b in audit['bodies']],
                        parked_vehicle_overlaps=[b['parked_vehicle_overlap'] for b in audit['bodies']])
                    surfaces=ground|{s for s in row['wheel_surfaces'] if s and s.startswith(('SM_Road_','SM_Floor_'))}
                    if not supported_ramp_contact(row,surfaces,clearance_m=None):
                        report['body_obstacle_contacts']+=1;raise RuntimeError(name+' native collision '+str(row))
                    baseline[name]=hit['time_stamp']
                m=moves.get(name)
                if m and name in dispatch.moves:
                    if m.command_id not in dispatch.departures and not any(overlaps(b,sources[name],.1) for b in boxes[name]):
                        dispatch.departed(m.command_id,name,m.visit_id,now,tail_clear=True)
                    if controller.executions[name].finished:
                        if abs(state.speed)>.1:raise RuntimeError('Arrival acknowledged while moving')
                        dispatch.arrived(m.command_id,name,m.visit_id,now,stopped=True)
                        print('STOPPED ARRIVAL',name,m.destination,flush=True)
            for m in dispatch.tick(now):
                if m.vehicle_id in controller.executions:raise RuntimeError('Smoke fixture permits one haul per vehicle only')
                plans[m.vehicle_id]=routes.plan(m);controller.activate(m.vehicle_id,now);moves[m.vehicle_id]=m
                print('DISPATCH MOVE',m.vehicle_id,m.origin,'->',m.destination,flush=True)
            driving={n:s for n,s in states.items() if n in controller.executions}
            commands=controller.tick(driving,now,body_boxes={n:boxes[n] for n in driving}) if driving else {}
            moving=sum(abs(s.speed)>.1 for s in states.values())
            report['maximum_moving_vehicles']=max(report['maximum_moving_vehicles'],moving)
            for name in pool:
                target,delta=commands.get(name,(0.,0.));omega=target*math.tan(delta)/2.3285 if abs(target)>.05 else 0
                adapter.send(ControlCommand(name,sequence,adapter.clock(),target,omega,ttl=2))
            sequence+=1
            trajectory.write(json.dumps(dict(t_s=now,positions={n:[s.x,s.y,s.heading,s.speed] for n,s in states.items()},
                task_states={n:a.state for n,a in dispatch.assignments.items()},node_holders=dispatch.holders))+'\n')
            completed=sum(s==WorkStatus.COMPLETE for s in dispatch.status.values())
            if now-last_status>=5:
                report.update(completed_tasks=completed,elapsed_s=now,task_states={n:a.state for n,a in dispatch.assignments.items()})
                (OUT/'physical_progress.json').write_text(json.dumps(report,indent=2)+'\n');last_status=now;trajectory.flush()
                print('TASK PROGRESS',completed,'/ 2','moving',moving,'states',report['task_states'],flush=True)
            if completed==2:
                report.update(status='PASS',completed_tasks=completed,elapsed_s=now,ue_physics_validated=True,
                    maximum_error_m=max(ex.maximum_error for ex in controller.executions.values()))
                break
            time.sleep(.02)
    except BaseException as exc:
        report.update(status='FAIL',completed_tasks=sum(s==WorkStatus.COMPLETE for s in dispatch.status.values()),failure=repr(exc),traceback=traceback.format_exc(),failure_audits=locals().get('audits',{}))
        print('TASK FAIL',exc,flush=True)
    finally:
        if trajectory:trajectory.close()
        if adapter:adapter.close()
        for name in pool:
            try:stop_monitor(name)
            except Exception:pass
        c.reset();c.simPause(True)
        try:
            fixture('RESTORE')
            for name,pose in home.items():c.simSetVehiclePose(pose,True,vehicle_name=name)
            stop_dir=ROOT/'runs/ue_scene/test_sites'
            (stop_dir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=names)))
            ue_script(c,'stop_test_vehicle_motion.py',stop_dir/'motion_stop.json')
        finally:c.simPause(False)
        time.sleep(.5)
        for name,enabled in apis.items():c.enableApiControl(enabled,vehicle_name=name)
        report.update(fleet_restored=max(math.dist(home_ue[n][:2],absolute_ue(c,n,origin)[0][:2]) for n in names)<.1,
            max_restored_speed_m_s=max(abs(c.getCarState(vehicle_name=n).speed) for n in names),
            api_restored=all(c.isApiControlEnabled(vehicle_name=n)==apis[n] for n in names),saved_maps_unchanged=map_fingerprints()==maps,
            dispatch_events=dispatch.events,road_events=controller.events)
        if not all(report[k] for k in ('fleet_restored','api_restored','saved_maps_unchanged')):report['status']='FAIL'
        (OUT/'physical_acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
        (OUT/'physical_progress.json').write_text(json.dumps(report,indent=2)+'\n')
    print('TASK DISPATCH',report['status'],report['completed_tasks'],'/ 2',flush=True)
    return 0 if report['status']=='PASS' else 1

if __name__=='__main__':raise SystemExit(main())
