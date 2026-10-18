#!/usr/bin/env python3
"""Persistent physical transport tasks with waiting and continuous dispatch.

Initial test staging is reversible; task execution uses throttle/steering only.
Representative queue and subsequent task; no exhaustive waiting-route testing.
"""
import json,math,time,traceback,hashlib,argparse
from pathlib import Path
from validate_ue_node_pairs import (ROOT,airsim,rear,AirSimAdapter,AirSimConfig,
    ControlCommand,VehicleState,FileBodyObserver,TransportMap,TransportController,
    envelope,observed_boxes,supported_ramp_contact,ue_script,absolute_ue,map_fingerprints)
from logistics_park.adapters.airsim import RPCTransport
from ue_demo_progress import DemoProgress
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork,WorkStatus
from logistics_park.runtime.task_execution import ServiceRouteGeometry,ContinuousTaskExecutor
from logistics_park.safety.collision import overlaps
from simulate_ue_continuous_tasks import cache_routes
from logistics_park.scheduling.wait_dependencies import wait_snapshot
from logistics_park.scheduling.traffic_routing import TrafficRouting
from continuous_run_sources import algorithm_fingerprints
from logistics_park.runtime.diagnostic_precheck import prefix_precheck_ready
from logistics_park.runtime.fleet_progress import FleetProgressWatch,task_progress,LocalProgressWatch,local_progress
from logistics_park.runtime.body_observer import FleetFileObserver
from logistics_park.runtime.geometry_integrity import geometry_fingerprints,native_dependency_fingerprints
from logistics_park.runtime.drivable_surface import DrivableSurface
from logistics_park.runtime.road_guard import RoadGuard
from logistics_park.runtime.surface_preflight import verify_service_surfaces
from logistics_park.runtime.drivable_audit import audit_frames
from logistics_park.runtime.safety_timing import MAX_SOURCE_STATE_AGE_S,MAX_BATCH_STATE_AGE_S,NATIVE_CONTROL_TIMEOUT_S,validate_native_policy

OUT=ROOT/'runs/continuous_tasks';OUT.mkdir(parents=True,exist_ok=True)
FIXTURE=ROOT/'runs/ue_node_pairs'

def main():
    global OUT
    FIXTURE.mkdir(parents=True,exist_ok=True)
    (ROOT/'runs/ue_scene/test_sites').mkdir(parents=True,exist_ok=True)
    parser=argparse.ArgumentParser();parser.add_argument('--scenario',default=str(ROOT/'config/runtime/industrialarea_ue_full_verified_20261007.json'));parser.add_argument('--diagnose-stall',action='store_true');parser.add_argument('--diagnostic-prefix',action='store_true');parser.add_argument('--time-scale',type=int,choices=(1,2,4));args=parser.parse_args()
    specpath=Path(args.scenario).resolve();spec=json.loads(specpath.read_text());OUT=ROOT/spec.get('output','runs/continuous_tasks');OUT.mkdir(parents=True,exist_ok=True)
    networkpath=ROOT/spec.get('network','config/topology/industrialarea_ue_service_network.json')
    world_time_scale=args.time_scale or spec.get('world_time_scale',4)
    native_safety=spec.get('native_safety',{})
    if native_safety:validate_native_policy(native_safety,world_time_scale)
    geometry_assets=geometry_fingerprints(spec.get('ue_geometry_asset_sha256',{}))
    native_assets=native_dependency_fingerprints(spec.get('ue_native_dependency_sha256',{}))
    c=airsim.CarClient(timeout_value=10)
    c.client.close();c.client=RPCTransport('127.0.0.1',41451,10)
    names=c.listVehicles()
    if c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in names):
        raise RuntimeError('Stationary unpaused fleet required')
    scene=ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json')
    if native_assets:
        native_dependency_fingerprints(native_assets,mappings=Path('/proc/'+str(scene['process_id'])+'/maps').read_text())
    if {v['name'] for v in scene['pie_vehicles']}!=set(names):raise RuntimeError('Actor fleet disagrees; restart PIE')
    origin=tuple(v/100 for v in scene['world_origin_cm'])
    home={n:c.simGetVehiclePose(vehicle_name=n) for n in names}
    home_ue={n:absolute_ue(c,n,origin)[0] for n in names}
    apis={n:c.isApiControlEnabled(vehicle_name=n) for n in names};maps=map_fingerprints()
    model=TransportMap(ROOT/spec['map'])
    network=ServiceNetwork(networkpath,blocked_edges=spec['blocked_candidate_edges'])
    if len(spec['positions'])>2:cache_routes(network)
    routes=ServiceRouteGeometry(network,model)
    pavement=None;surface_preflight=None
    if native_safety:
        surface_row=ue_script(c,'export_drivable_surface.py',ROOT/'runs/ue_scene/drivable_surface.json')
        pavement=DrivableSurface(surface_row)
        (OUT/'drivable_surface.json').write_text(json.dumps(surface_row))
    light=json.loads((OUT/'lightweight_acceptance.json').read_text())
    current_sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (specpath,ROOT/spec['map'],networkpath)}
    persistent_cycle=(str(light.get('failure','')).startswith('Persistent stationary waiting cycle')
                      and light.get('cycle_observation_s',0)>=15
                      and bool(light.get('wait_dependencies',{}).get('cycles'))
                      and bool(light.get('cycle_displacement_m'))
                      and all(v<.25 for v in light['cycle_displacement_m'].values()))
    stall_diagnostic=(args.diagnose_stall and light['status']=='FAIL'
                      and (light.get('failure')=="RuntimeError('Fleet stalled without moving vehicles')" or persistent_cycle)
                      and all(light.get(k)==0 for k in ('collision_count','unauthorized_shared_entry_count',
                          'local_unauthorized_entry_count','road_capacity_violation_steps')))
    prefix_diagnostic=args.diagnostic_prefix and prefix_precheck_ready(light,len(spec['positions']),spec.get('diagnostic_preflight_s'))
    if light['source_sha256']!=current_sources or (light['status']!='PASS' and not stall_diagnostic and not prefix_diagnostic):
        raise RuntimeError('Current lightweight precheck must pass, or explicit collision-free stall diagnostic required')
    if spec.get('traffic_routing',False) and light.get('algorithm_sha256')!=algorithm_fingerprints(ROOT):
        raise RuntimeError('Congestion precheck algorithm code has changed')
    if spec['map_sha256']!=maps:raise RuntimeError('Continuous fixture differs from saved maps')
    fleet=FleetRegistry()
    traffic=TrafficRouting(network,model,**spec.get('traffic_routing_options',{})) if spec.get('traffic_routing',False) else None
    dispatch=TaskDispatcher(network,fleet,service_s=5.,protect_committed_routes=spec.get('rolling_resources',False) or traffic is not None,traffic_routing=traffic,follow_to_busy_destination=spec.get('local_traffic',False),park_idle_vehicles=spec.get('park_idle_vehicles',False),offroad_busy_wait=spec.get('offroad_busy_wait',False),efficiency_policy=spec.get('efficiency_policy'),scheduling_options=spec.get('scheduling_options'))
    tasks=[(name,node,'staging') for name,node in spec['positions'].items()]
    work=spec['tasks']
    pool=[t[0] for t in tasks];plans={};initial={};visits={};moves={};sources={}
    for name,a,b in tasks:initial[name]=routes.stationary(a)
    controller=TransportController(model,plans,service_s=0,shared_resources=True,collect_at_destination=True,retain_finished_vehicles=True,rolling_resources=spec.get('rolling_resources',False),pipeline_shared=spec.get('traffic_routing',False),local_traffic=spec.get('local_traffic',False))
    if pavement:controller.road_guard=RoadGuard(pavement)
    executor=ContinuousTaskExecutor(dispatch,routes,controller)
    cfg=json.loads((ROOT/'config/runtime/industrialarea_ue_interwarehouse_1v_1cycles.json').read_text())['adapter']
    cfg.update(spec.get('adapter_options',{}))
    cfg.update(vehicle_ids=tuple(pool),origin_ue_m=origin,command_ttl_s=2,state_timeout_s=5.)
    if native_safety:cfg['state_timeout_s']=MAX_SOURCE_STATE_AGE_S
    observer=FileBodyObserver(ROOT/'runs/ue_scene',prefix='task',wheel_diagnostic_only=True)
    ground={a['name'] for a in scene['actors'] if a['name'].startswith('Traffic') and any(t in a['name'] for t in ('Ramp','Hump','ToCurb','ToHardstand','LaneTaper','SeamBridge'))}
    adapter=None;baseline={};trajectory=None
    hud=DemoProgress(c,len(spec['tasks']),len(pool)) if spec.get('ue_progress',False) else None
    report=dict(status='RUNNING',scope=spec.get('scope','Representative continuous transport'),
        expected_tasks=len(work),completed_tasks=0,maximum_moving_vehicles=0,body_obstacle_contacts=0,vehicle_collisions=0,ground_contact_samples=[],
        camera_changed=False,surface_preflight=surface_preflight,map_sha256=maps,ue_geometry_asset_sha256=geometry_assets,ue_native_dependency_sha256=native_assets,algorithm_sha256=algorithm_fingerprints(ROOT),source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (specpath,ROOT/spec['map'],networkpath)},
        requested_world_time_scale=world_time_scale,lightweight_status=light['status'],stall_diagnostic=stall_diagnostic,prefix_diagnostic=prefix_diagnostic,vehicle_count=len(pool),vehicles_removed_between_legs=False,teleports_during_tasks=0,ue_physics_validated=False,cargo_visual_effect_validated=False)
    def local_diagnostics(now,commands):
        local=controller.local_traffic
        if not local:return {}
        return dict(local_unauthorized_entry_count=controller.local_unauthorized_count,
            road_capacity_violation_steps=controller.capacity_violation_steps,conflict_windows={n:{k:(None if isinstance(v,float) and not math.isfinite(v) else v) for k,v in w.items()} for n,w in local.timing.windows.items()},
            timing_statistics=local.timing.statistics,commanded_speeds={n:v[0] for n,v in commands.items()},
            control_trace=getattr(controller,'control_trace',{}),
            destination_holds=dict(controller.destination_holds),
            berth_queue_intents=dict(dispatch.approach_intents),
            source_clearance_estimates=dict(dispatch.clearance_estimates),
            task_assignments={n:dict(task_id=a.work.task_id,state=a.state,operation=a.operation,
                source=dispatch.positions.get(n),target=dispatch._target(a)) for n,a in dispatch.assignments.items()},
            passage_yields={n:[list(k) for k in sorted(keys)] for n,keys in local.paused_by.items()},
            maneuver_holds=dict(local.maneuver_holds),yield_holds=dict(local.yield_holds),
            clearance_pauses={n:sorted(v) for n,v in local.clearance_pauses.items()},
            clearance_rights=[dict(owner=o,start=k,vehicle=n,end_m=r['end']) for (o,k,n),r in local.clearance_rights.items()],
            resume_priorities=sorted(local.resume_priorities),
            execution_details={n:dict(progress_m=ex.progress,finished=ex.finished,checkpoint_index=ex.checkpoint,
                checkpoint=ex.plan.checkpoints[ex.checkpoint] if ex.checkpoint<len(ex.plan.checkpoints) else None,
                edge_id=ex.plan.edges[ex.plan.edge_index(ex.progress)],
                reverse=ex.plan.reverse(ex.progress),destination=ex.plan.stops[-1] if ex.plan.stops else None,
                route_edges=list(ex.plan.edges),service_until=ex.service_until) for n,ex in controller.executions.items()},
            road_occupants={k:sorted(v) for k,v in controller.capacity.occupants.items() if v},
            road_commitments={k:sorted(v) for k,v in controller.capacity.committed.items() if v},
            braking_advice=getattr(local,'braking_advice',{}),local_denials=local.last_denials,passage_denials=local.passage_denials,control_denials=controller.control_denials,
            local_permissions=[dict(vehicle_id=n,start=start,end_m=lease['end'],
                progress_m=controller.executions[n].progress,
                distance_to_entry_m=lease['plan'].edge_starts[start]-controller.executions[n].progress,
                age_s=now-lease.get('created_at',now)) for (n,start),lease in local.passages.items()],
            ungranted_stop_lines={n:[dict(start=start,distance_m=entry-7.8-controller.executions[n].progress)
                for start,end,entry,exit_s in groups if exit_s+6>=controller.executions[n].progress
                and (n,start) not in local.passages] for n,groups in getattr(local,'action_groups',{}).items()})
    def console(command):
        if not c.simRunConsoleCommand(command):raise RuntimeError('UE rejected command')
    def fixture(action,vehicles=None):
        (FIXTURE/'fleet_request.json').write_text(json.dumps(dict(action=action,vehicles=vehicles or names,time_scale=world_time_scale,physics_substepping=spec.get('physics_substepping',False))))
        return ue_script(c,'node_pair_fleet_fixture.py',FIXTURE/'fleet_status.json')
    def stop_monitor(name):
        console("py import builtins,unreal; h=getattr(builtins,"+repr('task_monitor_'+name)+",None); unreal.unregister_slate_post_tick_callback(h) if h is not None else None; setattr(builtins,"+repr('task_monitor_'+name)+",None)")
    try:
        c.stopRecording()
        if hud:hud.start()
        ue_script(c,'prepare_dynamic_validation.py',ROOT/'runs/ue_scene/dynamic_editor_settings.json')
        # The fleet remains under full brake during fixture setup and initial
        # route compilation. Arm command deadlines just before first motion.
        fixture('INIT');adapter=AirSimAdapter(AirSimConfig(**cfg),OUT/'physical_adapter.jsonl',watchdog=False);adapter.open()
        if pavement:
            report['surface_preflight']=verify_service_surfaces(routes,pavement,progress=lambda done,poses,sweeps:
                c.simPrintLogMessage('Safety preflight: ',str(done)+' routes | '+str(poses)+' body poses',0))
        for name,a,b in tasks:
            p=initial[name];x,y=p.point(0);h=p.heading(0)
            target=[x+1.16425*math.cos(h),y+1.16425*math.sin(h),.894]
            hp=home[name].position;world=home_ue[name]
            pose=airsim.Pose(airsim.Vector3r(hp.x_val+target[0]-world[0],hp.y_val+target[1]-world[1],hp.z_val-target[2]+world[2]),airsim.to_quaternion(0,0,h))
            c.simPause(True)
            try:
                fixture('ACTIVATE',[name]);c.simSetVehiclePose(pose,True,vehicle_name=name)
            finally:
                if not spec.get('batch_observer'):c.simPause(False)
            v=fleet.adopt_present_vehicle(name,0,fully_inside=True,resources=[]);visits[name]=v.visit_id
            dispatch.place_vehicle(name,v.visit_id,a,0,stopped=True)
            baseline[name]=c.simGetCollisionInfo(vehicle_name=name).time_stamp
            sources[name]=envelope(VehicleState(name,x,y,h))
            if spec.get('batch_observer'):
                if math.dist(absolute_ue(c,name,origin)[0][:2],target[:2])>.3:raise RuntimeError('Invalid staged start '+name)
                continue
            prefix='task_'+name;live=ROOT/'runs/ue_scene'/f'{prefix}_live.json';live.unlink(missing_ok=True)
            console("py import builtins,runpy; builtins.short_cargo_monitor_request="+repr(dict(vehicle=name,prefix=prefix,handle='task_monitor_'+name,exclude_inactive_fixture=True,cache_frame_boxes=len(pool)>2))+"; runpy.run_path("+repr(str(ROOT/'scripts/ue/monitor_short_cargo.py'))+"); del builtins.short_cargo_monitor_request")
            deadline=time.monotonic()+2
            while not live.exists() and time.monotonic()<deadline:time.sleep(.02)
            if not live.exists() or math.dist(json.loads(live.read_text())['truck_ue_m'][:2],target[:2])>.3:
                raise RuntimeError('Invalid staged start '+name)
        if spec.get('batch_observer'):
            # Stage the whole batch while paused; a pose teleport can leave
            # stale chassis velocity until the next physics update. Clear it
            # once, before any task is issued, then observe the settled fleet.
            stop_dir=ROOT/'runs/ue_scene/test_sites'
            (stop_dir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=pool)))
            ue_script(c,'stop_test_vehicle_motion.py',stop_dir/'motion_stop.json')
            c.simPause(False)
            time.sleep(.15)
            live=ROOT/'runs/ue_scene/flow_fleet_live.json';live.unlink(missing_ok=True)
            console("py import builtins,runpy; builtins.fleet_monitor_request="+repr(dict(vehicles=pool,road_history=str(OUT/'native_road_history.jsonl') if pavement else None))+"; runpy.run_path("+repr(str(ROOT/'scripts/ue/monitor_fleet.py'))+")")
            observer=FleetFileObserver(live,max_age_s=.3 if native_safety else 1.2)
            adapter.snapshot_provider=observer.snapshot;adapter.last_source={};adapter.source_advanced={}
        measured=adapter.read_states()
        for name,a,b in tasks:
            audit,_=observer.read(name)
            executor.attach(name,a,rear(measured[name]),0,body_boxes=observed_boxes(audit))
        for i,t in enumerate(work):dispatch.submit(TransportWork('transport-'+str(i),t[0],t[1]))
        executor.prepare_schedule({name:rear(measured[name]) for name in pool})
        adapter.start_watchdog_after_staging(native_guard=bool(native_safety))
        replan_hold_after=None;consecutive_overruns=0
        begin=time.monotonic();last_status=0;last_diagnostic=-1.;sequence=1;last_motion=0;last_cycle=0;cycles=[]
        progress_watch=FleetProgressWatch(spec.get('early_stall_check_s',15),spec.get('early_stall_displacement_m',.25)) if spec.get('early_stall_check_s') else None
        local_watch=LocalProgressWatch(spec.get('local_stall_check_s',20),spec.get('local_wait_limit_s',90)) if spec.get('local_traffic') else None
        last_local_check=-1.
        trajectory=(OUT/'physical_trajectory.jsonl').open('w')
        while True:
            now=time.monotonic()-begin
            if now>spec.get('physical_limit_s',900):raise RuntimeError('Continuous task test time limit')
            measured=adapter.read_states();states={n:rear(measured[n]) for n in pool};boxes={};audits={}
            if replan_hold_after is not None:
                if min(s.timestamp for s in states.values())<=replan_hold_after:
                    time.sleep(.02);continue
                controller.road_guard.acknowledge_brake();replan_hold_after=None
            if pavement:
                controller.ground_heights={n:adapter.telemetry[n]['ue_z_m'] for n in states}
                for n,row in observer.latest['states'].items():
                    if not row['native_guard_active'] or not row['native_timed_controls_active'] or abs(row['native_control_timeout_s']-NATIVE_CONTROL_TIMEOUT_S)>1e-6:
                        raise RuntimeError('Native safety lease not active: '+n)
                    if row['native_rejected_controls']:
                        raise RuntimeError('Native safety rejected stale or invalid control: '+n)
            for name,state in states.items():
                audit,errors=observer.read(name);audits[name]=audit;boxes[name]=observed_boxes(audit)
                if errors:
                    report['body_obstacle_contacts']+=sum(bool(b['static_sweep_hit']) for b in audit['bodies'])
                    report['vehicle_collisions']+=sum(bool(b['parked_vehicle_overlap']) for b in audit['bodies'])
                    raise RuntimeError(name+' body audit '+str(errors))
                hit=adapter.telemetry[name]['collision']
                if hit['has_collided'] and hit['time_stamp']!=baseline[name]:
                    if spec.get('batch_observer'):
                        console("py import builtins; builtins.flow_wheel_request="+repr(name))
                        wheel_path=ROOT/'runs/ue_scene'/('flow_wheels_'+name+'.json')
                        sampled=ue_script(c,'sample_fleet_wheels.py',wheel_path)
                        if time.monotonic()-sampled['monotonic_s']>1.2:raise RuntimeError('stale wheel support sample')
                        audit['wheels']=sampled['wheels']
                    row=dict(collision=hit['object_name'],collision_normal=[hit['normal'][k] for k in ('x_val','y_val','z_val')],
                        collision_penetration_m=hit['penetration_depth'],wheel_clearances_m=[w['clearance_m'] for w in audit['wheels']],
                        wheel_surfaces=[w['road_actor'] for w in audit['wheels']],body_hits=[b['static_sweep_hit'] for b in audit['bodies']],
                        parked_vehicle_overlaps=[b['parked_vehicle_overlap'] for b in audit['bodies']])
                    surfaces=ground|{s for s in row['wheel_surfaces'] if s and s.startswith(('SM_Road_','SM_Floor_'))}
                    if not supported_ramp_contact(row,surfaces,clearance_m=None):
                        report['body_obstacle_contacts']+=1;raise RuntimeError(name+' native collision '+str(row))
                    report['ground_contact_samples'].append(dict(t_s=now,vehicle_id=name,**row))
                    baseline[name]=hit['time_stamp']
            commands=executor.tick(states,now,body_boxes=boxes)
            if native_safety and adapter.clock()-min(s.timestamp for s in states.values())>MAX_BATCH_STATE_AGE_S:
                replan_hold_after=adapter.hold_for_replan();consecutive_overruns+=1
                report['control_overrun_holds']=report.get('control_overrun_holds',0)+1
                if consecutive_overruns>=3:raise RuntimeError('Control computation repeatedly exceeds fresh-state batch budget')
                time.sleep(.1);continue
            consecutive_overruns=0
            if progress_watch:
                token,deadline=task_progress(executor,now)
                report['progress_check']=progress_watch.observe(states,now,token,scheduled_activity_until=deadline)
                if report['progress_check']['stalled']:raise RuntimeError('Fleet effective progress stalled: '+str(report['progress_check']))
            if local_watch and now-last_local_check>=1.:
                report['local_progress_check'],report['wait_dependencies']=local_progress(executor,states,now,local_watch)
                last_local_check=now
                if report['local_progress_check']['stalled']:
                    report.update(local_diagnostics(now,commands))
                    raise RuntimeError('Local effective progress stalled: '+str(report['local_progress_check']))
            moving=sum(abs(s.speed)>.1 for s in states.values())
            if moving:last_motion=now
            if now-last_motion>spec.get('stall_limit_s',900):raise RuntimeError('Fleet stalled without moving vehicles')
            cycles.append(time.monotonic()-begin-last_cycle);last_cycle=time.monotonic()-begin
            report['maximum_moving_vehicles']=max(report['maximum_moving_vehicles'],moving)
            for name in pool:
                target,delta=commands.get(name,(0.,0.));omega=target*math.tan(delta)/2.3285 if abs(target)>.05 else 0
                adapter.send(ControlCommand(name,sequence,adapter.clock(),target,omega,ttl=2,steering_rad=delta))
            if pavement:controller.road_guard.acknowledge_commands({n:adapter.drive_direction[n]*adapter.applied_speed[n] for n in pool})
            sequence+=1
            trajectory.write(json.dumps(dict(t_s=now,rear_z_m={n:adapter.telemetry[n]['ue_z_m'] for n in states},positions={n:[s.x,s.y,s.heading,s.speed] for n,s in states.items()},
                task_states={n:a.state for n,a in dispatch.assignments.items()},node_holders=dispatch.holders))+'\n')
            if now-last_diagnostic>=1.:
                diagnostic=local_diagnostics(now,commands)
                with (OUT/'physical_control_diagnostics.jsonl').open('a') as log:
                    log.write(json.dumps(dict(t_s=now,positions={n:[s.x,s.y,s.heading,s.speed] for n,s in states.items()},**diagnostic))+'\n')
                last_diagnostic=now
            completed=sum(s==WorkStatus.COMPLETE for s in dispatch.status.values())
            if hud:
                parked=sum(dispatch.network.services[node]['kind']=='waiting' and abs(states[name].speed)<=.1
                           for name,node in dispatch.positions.items() if name in states)
                hud.update('PARKING' if completed==len(work) else 'RUNNING',completed,moving,parked,now)
            if now-last_status>=5:
                report.update(completed_tasks=completed,elapsed_s=now,mean_control_cycle_s=sum(cycles)/len(cycles),maximum_control_cycle_s=max(cycles),task_states={n:a.state for n,a in dispatch.assignments.items()})
                report.update(local_diagnostics(now,commands))
                (OUT/'physical_progress.json').write_text(json.dumps(report,indent=2)+'\n');last_status=now;trajectory.flush()
                print('CONTINUOUS PROGRESS',completed,'/',len(work),'moving',moving,'states',report['task_states'],flush=True)
            if completed==len(work) and dispatch.ready_to_finish():
                report.update(status='PASS',completed_tasks=completed,elapsed_s=now,ue_physics_validated=True,
                    maximum_error_m=controller.maximum_error,parking_complete=dispatch.ready_to_finish(),parked_nodes={v:n for v,n in dispatch.positions.items() if dispatch.network.services[n]['kind']=='waiting'},waiting_nodes_visited=[m.destination for m in executor.completed_moves if m.waiting])
                if not report['waiting_nodes_visited'] and not spec.get('local_traffic'):raise RuntimeError('Representative task did not exercise waiting')
                break
            time.sleep(.02)
    except BaseException as exc:
        report.update(status='FAIL',elapsed_s=locals().get('now',0),completed_tasks=sum(s==WorkStatus.COMPLETE for s in dispatch.status.values()),failure=repr(exc),traceback=traceback.format_exc(),failure_audits=locals().get('audits',{}),failure_states={n:dict(x=s.x,y=s.y,heading=s.heading,speed=s.speed) for n,s in locals().get('states',{}).items()})
        report['wait_dependencies']=wait_snapshot(executor,locals().get('states',{})) if locals().get('states') else {}
        print('TASK FAIL',exc,flush=True)
    finally:
        report['cleanup_complete']=False
        try:
            if trajectory:trajectory.close()
            if adapter:adapter.close()
            if spec.get('batch_observer'):
                try:console("py import builtins,unreal; h=getattr(builtins,'flow_fleet_monitor',None); unreal.unregister_slate_post_tick_callback(h) if h is not None else None; builtins.flow_fleet_monitor=None")
                except Exception:pass
            for name in pool:
                try:stop_monitor(name)
                except Exception:pass
            try:console("py import builtins; builtins.fleet_body_frame_cache=None")
            except Exception:pass
            c.simPause(True)
            fixture('FREEZE')
            c.reset();c.simPause(True)
            try:
                fixture('RESTORE')
                # UE fixture restores saved world poses; pre-reset local NED poses
                # are no longer reliable after AirSim rebuilds per-vehicle origins.
                stop_dir=ROOT/'runs/ue_scene/test_sites'
                (stop_dir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=names)))
                ue_script(c,'stop_test_vehicle_motion.py',stop_dir/'motion_stop.json')
            finally:c.simPause(False)
            time.sleep(.5)
            for name,enabled in apis.items():c.enableApiControl(enabled,vehicle_name=name)
            report.update(fleet_restored=max(math.dist(home_ue[n][:2],absolute_ue(c,n,origin)[0][:2]) for n in names)<.1,
                max_restored_speed_m_s=max(abs(c.getCarState(vehicle_name=n).speed) for n in names),
                api_restored=all(c.isApiControlEnabled(vehicle_name=n)==apis[n] for n in names),saved_maps_unchanged=map_fingerprints()==maps,
                dispatch_events=dispatch.events,road_events=controller.events,
                wait_dependencies=wait_snapshot(executor,locals().get('states',{})),
                maximum_error_m=controller.maximum_error,shared_owners=dict(controller.shared_scheduler.owners) if controller.shared_scheduler else {},
                task_completion_by_vehicle={n:sum(e['event']=='TRANSPORT_COMPLETE' and e.get('vehicle_id')==n for e in dispatch.events) for n in pool},
                task_assigned_vehicles=sorted({e['vehicle_id'] for e in dispatch.events if e['event']=='TASK_DISPATCHED'}),
                throughput_tasks_per_wall_hour=sum(s==WorkStatus.COMPLETE for s in dispatch.status.values())/max(.001,report.get('elapsed_s',0))*3600)
            report['saved_geometry_assets_unchanged']=geometry_fingerprints(geometry_assets)==geometry_assets
            report['native_dependencies_unchanged']=native_dependency_fingerprints(native_assets)==native_assets
            if pavement and (OUT/'native_road_history.jsonl').exists():
                with (OUT/'native_road_history.jsonl').open() as stream:
                    full_road_audit=audit_frames((json.loads(line) for line in stream),pavement)
                (OUT/'drivable_surface_audit.json').write_text(json.dumps(full_road_audit,indent=2)+'\n')
                report['drivable_surface_audit']={k:v for k,v in full_road_audit.items() if k!='flags'}
                if full_road_audit['status']!='PASS':report['status']='FAIL'
            elif pavement:report['status']='FAIL';report['road_audit_error']='Native full-route road history missing'
            if not all(report[k] for k in ('fleet_restored','api_restored','saved_maps_unchanged','saved_geometry_assets_unchanged','native_dependencies_unchanged')):report['status']='FAIL'
            report['cleanup_complete']=True
        except Exception as cleanup_error:
            report.update(status='FAIL',cleanup_error=repr(cleanup_error),ue_physics_validated=False)
        finally:
            report.update(dispatch_events=dispatch.events,road_events=controller.events,maximum_error_m=controller.maximum_error)
            report.update(local_diagnostics(locals().get('now',0),locals().get('commands',{})))
            if pavement:report['road_guard']=dict(command_evaluations=controller.road_guard.checked,limited_commands=controller.road_guard.limited,braking_recoveries=controller.road_guard.braking_recoveries,latest=controller.road_guard.trace)
            report['local_events']=controller.local_traffic.events if controller.local_traffic else []
            report['timing_events']=controller.local_traffic.timing.events if controller.local_traffic else []
            (OUT/'physical_acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
            (OUT/'physical_progress.json').write_text(json.dumps(report,indent=2)+'\n')
    if hud:
        hud.update(report['status'],report['completed_tasks'],0,
                   len(report.get('parked_nodes',{})),report.get('elapsed_s',0),
                   failure=report.get('failure',''),force=True)
    print('CONTINUOUS TASKS',report['status'],report['completed_tasks'],'/',len(work),flush=True)
    return 0 if report['status']=='PASS' else 1

if __name__=='__main__':
    from logistics_park.runtime.run_lock import exclusive_run
    with exclusive_run(ROOT/'runs/ue_scene/continuous_physical.lock'):
        raise SystemExit(main())
