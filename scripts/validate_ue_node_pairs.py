#!/usr/bin/env python3
"""Rolling physical verification of 132 loading/unloading pairs with 12 trucks."""
import argparse,hashlib,json,math,sys,time,traceback
from pathlib import Path
import airsim
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig
from logistics_park.common.messages import ControlCommand,VehicleState
from logistics_park.runtime.body_observer import FileBodyObserver
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.transport_control import TransportController,envelope
from logistics_park.runtime.node_pair_validation import can_stage,clear_collected,clear_aborted
from logistics_park.runtime.route_coverage import coverage_result
from logistics_park.runtime.ue_transport import observed_boxes
from logistics_park.runtime.ground_contact import supported_ramp_contact
from simulate_ue_node_pairs import plans_for
from ue_trailer_driving_check import ue_script
from ue_dynamic_crossing_check import absolute_ue
from ue_junction_check import teleport
from validate_ue_loading_access import map_fingerprints

def rear(state):
    return VehicleState(state.vehicle_id,state.x-1.16425*math.cos(state.heading),state.y-1.16425*math.sin(state.heading),state.heading,speed=state.speed,timestamp=state.timestamp)


OUT=ROOT/'runs/ue_node_pairs';OUT.mkdir(parents=True,exist_ok=True)


def route_fingerprint(plan):
    fields=('centerline','reverse','kind','maneuver','speed_limit','straight_until_m','maximum_path_error_m')
    geometry=[{k:plan.model.edges[e].get(k) for k in fields} for e in plan.edges]
    return hashlib.sha256(json.dumps(dict(geometry=geometry,checkpoints=plan.checkpoints),sort_keys=True).encode()).hexdigest()


def main():
    global OUT
    parser=argparse.ArgumentParser();parser.add_argument('--resume',action='store_true')
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--smoke',action='store_true')
    modes.add_argument('--pairs',nargs='+',help='Diagnostic subset; never replaces the full sweep report')
    modes.add_argument('--waiting-access',action='store_true',help='Only Q1-Q12 local ingress/stop/egress')
    modes.add_argument('--road-scan',action='store_true',help='Deduplicated road/transition scan; separate from 132 end-to-end results')
    parser.add_argument('--scenario',type=Path,help='Custom local waiting scan manifest')
    parser.add_argument('--output-dir',type=Path,help='Isolated local scan evidence directory')
    parser.add_argument('--time-scale',type=float,default=2)
    args=parser.parse_args()
    if (args.scenario or args.output_dir) and not args.waiting_access:raise RuntimeError('Custom paths require --waiting-access')
    if args.output_dir:OUT=args.output_dir.resolve();OUT.mkdir(parents=True,exist_ok=True)

    if not 1<=args.time_scale<=4:raise RuntimeError('Physical test time scale must be between 1 and 4')
    scenario_path=args.scenario or ROOT/('config/runtime/industrialarea_ue_waiting_scan.json' if args.waiting_access else 'config/runtime/industrialarea_ue_node_pair_tests.json')
    spec=json.loads(scenario_path.read_text())
    if args.waiting_access and (spec['service_network_sha256']!=hashlib.sha256((ROOT/spec.get('service_network','config/topology/industrialarea_ue_service_network.json')).read_bytes()).hexdigest() or spec['map_sha256']!=map_fingerprints()):
        raise RuntimeError('Waiting manifest differs from current source network or saved maps')
    topology_sha=hashlib.sha256((ROOT/spec['map']).read_bytes()).hexdigest()
    scenario_sha=hashlib.sha256(scenario_path.read_bytes()).hexdigest()
    scan=None
    if args.road_scan:
        scan=json.loads((ROOT/'config/runtime/industrialarea_ue_road_scan.json').read_text())
        if scan['topology_sha256']!=topology_sha or scan['scenario_sha256']!=scenario_sha:
            raise RuntimeError('Road scan is stale; run build_ue_road_scan.py again')
        from logistics_park.runtime.route_coverage import coverage_sets
        actual=coverage_sets(json.loads((ROOT/spec['map']).read_text()),spec['jobs'])
        if {k:set(v) for k,v in scan['requirements_by_pair'].items()}!=actual:
            raise RuntimeError('Road scan requirements disagree with current routes')
        if not scan['selected_pair_ids'] or not set(scan['selected_pair_ids'])<=set(actual):
            raise RuntimeError('Invalid road scan route selection')
        required=set().union(*actual.values())
        selected=set().union(*(actual[k] for k in scan['selected_pair_ids']))
        if required!=selected:raise RuntimeError('Road scan omits required roads or transitions')
    light=json.loads((OUT/('waiting_lightweight_acceptance.json' if args.waiting_access else 'scan_lightweight_acceptance.json' if scan else 'lightweight_acceptance.json')).read_text())
    expected_light=len(spec['jobs']) if args.waiting_access else scan['selected_route_count'] if scan else 132
    if (light['status']!='PASS' or light['completed_pairs']!=expected_light
            or light.get('topology_sha256')!=topology_sha):raise RuntimeError('Current-route lightweight precheck must pass')
    if (scan or args.waiting_access) and (light.get('scenario_sha256')!=scenario_sha or
            {r['pair_id'] for r in light['results'] if r['status']=='PASS'}!=(set(scan['selected_pair_ids']) if scan else {j['pair_id'] for j in spec['jobs']})):
        raise RuntimeError('Lightweight precheck does not match the selected scan jobs')
    c=airsim.CarClient(timeout_value=10);names=c.listVehicles()
    if c.simIsPause() or any(abs(c.getCarState(vehicle_name=n).speed)>.1 for n in names):raise RuntimeError('Stationary unpaused fleet required')
    ue_script(c,'prepare_dynamic_validation.py',ROOT/'runs/ue_scene/dynamic_editor_settings.json')
    scene=ue_script(c,'export_scene.py',ROOT/'runs/ue_scene/scene.json');origin=[v/100 for v in scene['world_origin_cm']]
    if {v['name'] for v in scene['pie_vehicles']}!=set(names):
        raise RuntimeError('UE actor fleet and AirSim fleet disagree; restart PIE before testing')
    home={n:c.simGetVehiclePose(vehicle_name=n) for n in names};apis={n:c.isApiControlEnabled(vehicle_name=n) for n in names}
    home_xy={n:absolute_ue(c,n,origin)[0][:2] for n in names};maps=map_fingerprints()
    home_ue={n:absolute_ue(c,n,origin)[0] for n in names}
    pool=[f'Car_{i//3}_{i%3}' for i in range(12)]
    ground={a['name'] for a in scene['actors'] if a['name'].startswith('Traffic') and any(t in a['name'] for t in ('Ramp','Hump','ToCurb','ToHardstand','LaneTaper','SeamBridge'))}
    if spec.get('parking_static_probe_sha256'):
        probe_path=ROOT/'runs/ue_scene/station_siting/parking_expansion_probe.json'
        probe=json.loads(probe_path.read_text())
        if hashlib.sha256(probe_path.read_bytes()).hexdigest()!=spec['parking_static_probe_sha256'] or probe['status']!='PASS':raise RuntimeError('Parking static probe is stale or unsafe')
        ground.update(s for row in probe['rows'] for phase in row['phases'].values() for s in phase['ground_surfaces'])
    model=TransportMap(ROOT/spec['map']);templates=plans_for(model,spec['jobs']);plans={}
    (OUT/('topology_'+topology_sha+'.json')).write_bytes((ROOT/spec['map']).read_bytes())
    (OUT/('scenario_'+scenario_sha+'.json')).write_text(json.dumps(spec,indent=2)+'\n')
    controller=TransportController(model,plans,service_s=spec.get('service_s',.3),shared_resources=True,collect_at_destination=True)
    cfg=json.loads((ROOT/'config/runtime/industrialarea_ue_interwarehouse_1v_1cycles.json').read_text())['adapter']
    # Staging a truck deliberately blocks the control loop for ~1-2 s (UE console
    # script plus body-monitor start), and one twelve-vehicle state batch costs up
    # to ~1 s of simGetObjectPose round trips, so the 1.2 s default would expire
    # during that intentional pause.  A genuinely frozen feed is still rejected by
    # the source-timestamp check inside read_states.
    cfg.update(vehicle_ids=tuple(pool),origin_ue_m=tuple(origin),command_ttl_s=2,state_timeout_s=5.)
    adapter=None;observer=FileBodyObserver(ROOT/'runs/ue_scene',prefix='nodepairs',wheel_diagnostic_only=True)
    pending=sorted(spec['jobs'],key=lambda j:-j['distance_m']);active={};baseline={};results=[];sequence=1
    if args.smoke:
        selected={'W5-loading__W6-loading','W1-loading__W1-unloading'}
        pending=[j for j in spec['jobs'] if j['pair_id'] in selected]
    if args.pairs:
        selected=set(args.pairs)
        unknown=selected-{j['pair_id'] for j in spec['jobs']}
        if unknown:raise RuntimeError('Unknown diagnostic pairs: '+str(unknown))
        pending=[j for j in pending if j['pair_id'] in selected]
    if scan:
        selected=set(scan['selected_pair_ids'])
        pending=[j for j in pending if j['pair_id'] in selected]
    expected=len(pending)
    prefix='waiting_' if args.waiting_access else 'scan_' if scan else 'diagnostic_' if args.pairs else 'smoke_' if args.smoke else ''
    previous=OUT/(prefix+'acceptance.json')
    progress_file=OUT/(prefix+'progress.json')
    if args.resume and previous.exists():
        old=json.loads(previous.read_text())
        if old.get('map_sha256')!=maps:raise RuntimeError('Resume evidence belongs to a different saved scene')
        for r in old['results']:
            same_sources=old.get('topology_sha256')==topology_sha and old.get('scenario_sha256')==scenario_sha
            same_route=r.get('route_sha256')==route_fingerprint(templates[r['pair_id']])
            if (r['pair_id'] in {j['pair_id'] for j in pending}
                    and r['status']=='PASS' and (same_sources or same_route)):results.append(r)
        done={r['pair_id'] for r in results};pending=[j for j in pending if j['pair_id'] not in done]
    if previous.exists():
        (OUT/('acceptance_before_'+str(time.time_ns())+'.json')).write_bytes(previous.read_bytes())
    report=dict(status='RUNNING',expected_pairs=expected,results=results,camera_changed=False,map_sha256=maps,
        topology_sha256=topology_sha,scenario_sha256=scenario_sha,
        requested_world_time_scale=args.time_scale,
        scope='Node-to-node driving only; waiting nodes excluded as endpoints; completed trucks reversibly hidden with collision and physics disabled',
        maximum_moving_vehicles=0,body_obstacle_contacts=0,vehicle_collisions=0)
    if args.waiting_access:report.update(scope=spec['scope'],expected_waiting_nodes=spec['expected_waiting_nodes'])
    if scan:
        report.update(scope=scan['scope'],source_route_count=scan['source_route_count'],
                      coverage_manifest_sha256=hashlib.sha256((ROOT/'config/runtime/industrialarea_ue_road_scan.json').read_bytes()).hexdigest(),
                      **coverage_result(scan,results))
    def console(text):
        if not c.simRunConsoleCommand(text):raise RuntimeError('UE rejected command')
    def fixture(action,vehicles=None):
        (ROOT/'runs/ue_node_pairs/fleet_request.json').write_text(json.dumps(dict(action=action,vehicles=vehicles or names,time_scale=args.time_scale,output_dir=str(OUT))))
        return ue_script(c,'node_pair_fleet_fixture.py',OUT/'fleet_status.json')
    def stop_monitor(name):
        console("py import builtins,unreal; h=getattr(builtins,"+repr('nodepairs_monitor_'+name)+",None); unreal.unregister_slate_post_tick_callback(h) if h is not None else None; setattr(builtins,"+repr('nodepairs_monitor_'+name)+",None)")
    def counts():
        value=dict(attempted_pairs=len(results),completed_pairs=sum(r['status']=='PASS' for r in results),
                   failed_pairs=sum(r['status']=='FAIL' for r in results))
        if scan:value.update(coverage_result(scan,results))
        return value
    def fail_pair(name,reason,now,*,audit=None,native=None):
        if name not in active:return
        assignment=active[name];job=assignment['job'];ex=controller.executions[name]
        state=states.get(name)
        result=dict(pair_id=job['pair_id'],origin=job['origin'],destination=job['destination'],status='FAIL',
            reason=str(reason),vehicle=name,duration_s=now-assignment['start'],maximum_error_m=ex.maximum_error,
            route_sha256=route_fingerprint(plans[name]),ue_physics_validated=False,
            progress_m=ex.progress,edge_id=ex.plan.edges[ex.plan.edge_index(ex.progress)],
            state=None if state is None else dict(x=state.x,y=state.y,heading=state.heading,speed=state.speed),
            body_audit=audit,native_collision=native)
        fixture('FREEZE',[name]);stop_monitor(name)
        clear_aborted(controller,name,now,physically_removed=True)
        states.pop(name,None);boxes.pop(name,None);audits.pop(name,None);del active[name]
        results.append(result);report.update(counts())
        progress_file.write_text(json.dumps(report,indent=2)+'\n')
        print('RECORDED ISSUE',job['pair_id'],reason,'attempted',len(results),'/',expected,flush=True)
    try:
        fixture('INIT')
        adapter=AirSimAdapter(AirSimConfig(**cfg),OUT/(prefix+'adapter.jsonl'));adapter.open()
        begin=time.monotonic();last_stage=last_status=0
        trajectory=(OUT/(prefix+'trajectory.jsonl')).open('w')
        while pending or active:
            now=time.monotonic()-begin
            if now>18000:raise RuntimeError('Five-hour test limit')
            if now-last_stage>.5:
                last_stage=now
                for name in pool:
                    if name in active:continue
                    job=next((j for j in pending if can_stage(controller,templates[j['pair_id']])),None)
                    if job is None:continue
                    p=templates[job['pair_id']];x,y=p.point(0);h=p.heading(0)
                    target=[x+1.16425*math.cos(h),y+1.16425*math.sin(h),.894]
                    initial=home[name].position;world=home_ue[name]
                    pose=airsim.Pose(airsim.Vector3r(initial.x_val+target[0]-world[0],
                        initial.y_val+target[1]-world[1],initial.z_val-target[2]+world[2]),
                        airsim.to_quaternion(0,0,h))
                    paused=c.simIsPause();c.simPause(True)
                    try:
                        fixture('ACTIVATE',[name])
                        c.simSetVehiclePose(pose,True,vehicle_name=name)
                    finally:c.simPause(paused)
                    plans[name]=p;controller.activate(name,now)
                    controller.boxes[name]=[envelope(VehicleState(name,x,y,h))]
                    monitor_prefix='nodepairs_'+name;handle='nodepairs_monitor_'+name
                    live=ROOT/'runs/ue_scene'/f'{monitor_prefix}_live.json';live.unlink(missing_ok=True)
                    console("py import builtins,runpy; builtins.short_cargo_monitor_request="+repr(dict(vehicle=name,prefix=monitor_prefix,handle=handle,exclude_inactive_fixture=True))+"; runpy.run_path("+repr(str(ROOT/'scripts/ue/monitor_short_cargo.py'))+"); del builtins.short_cargo_monitor_request")
                    deadline=time.monotonic()+1
                    while not live.exists() and time.monotonic()<deadline:time.sleep(.02)
                    if not live.exists():raise RuntimeError('Body monitor did not start for '+name)
                    placed=json.loads(live.read_text())['truck_ue_m']
                    if math.dist(placed[:2],target[:2])>.3:raise RuntimeError('Incorrect fixture start pose '+name+': '+str(placed))
                    pending.remove(job);active[name]=dict(job=job,start=now,maximum_error_m=0,peak_speed_m_s=0)
                    baseline[name]=c.simGetCollisionInfo(vehicle_name=name).time_stamp
                    print('START',name,job['pair_id'],'remaining',len(pending),flush=True)
                    break  # Keep control updates frequent while admitting the next vehicle.
            measured=adapter.read_states();states={n:rear(measured[n]) for n in active};audits={};boxes={}
            for name in list(states):
                if name not in active:continue
                state=states[name]
                audit,errors=observer.read(name);audits[name]=audit;boxes[name]=observed_boxes(audit)
                if errors:
                    report['failed_body_audit']=audit
                    report['body_obstacle_contacts']+=sum(bool(b['static_sweep_hit']) for b in audit['bodies'])
                    report['vehicle_collisions']+=sum(bool(b['parked_vehicle_overlap']) for b in audit['bodies'])
                    contacts={b['parked_vehicle_overlap'] for b in audit['bodies'] if b['parked_vehicle_overlap']}
                    fail_pair(name,str(errors),now,audit=audit)
                    for other in contacts:fail_pair(other,'Vehicle contact with '+name,now,audit=audits.get(other))
                    continue
                hit=adapter.telemetry[name]['collision']
                if hit['has_collided'] and hit['time_stamp']!=baseline[name]:
                    row=dict(collision=hit['object_name'],collision_normal=[hit['normal'][k] for k in ('x_val','y_val','z_val')],
                        collision_penetration_m=hit['penetration_depth'],wheel_clearances_m=[w['clearance_m'] for w in audit['wheels']],
                        wheel_surfaces=[w['road_actor'] for w in audit['wheels']],body_hits=[b['static_sweep_hit'] for b in audit['bodies']],
                        parked_vehicle_overlaps=[b['parked_vehicle_overlap'] for b in audit['bodies']])
                    surfaces=ground|{s for s in row['wheel_surfaces'] if s and s.startswith(('SM_Road_','SM_Floor_'))}
                    if not supported_ramp_contact(row,surfaces,clearance_m=None):
                        report['body_obstacle_contacts']+=1
                        fail_pair(name,'Native collision '+hit['object_name'],now,audit=audit,native=row)
                        continue
                    baseline[name]=hit['time_stamp']
                active[name]['peak_speed_m_s']=max(active[name]['peak_speed_m_s'],abs(state.speed))
            while True:
                try:
                    commands=controller.tick(states,now,body_boxes=boxes)
                    break
                except RuntimeError as exc:
                    affected=[n for n in active if n in str(exc).split()]
                    if not affected:affected=list(active)
                    if not affected:raise
                    if 'vehicle envelope overlap' in str(exc):report['vehicle_collisions']+=1
                    for name in affected:fail_pair(name,str(exc),now,audit=audits.get(name))
            moving=sum(abs(s.speed)>.1 for s in states.values());report['maximum_moving_vehicles']=max(report['maximum_moving_vehicles'],moving)
            for name in pool:
                target,delta=commands.get(name,(0.,0.));omega=target*math.tan(delta)/2.3285 if abs(target)>.05 else 0
                adapter.send(ControlCommand(name,sequence,adapter.clock(),target,omega,ttl=2))
            sequence+=1
            trajectory.write(json.dumps(dict(t_s=now,positions={n:[s.x,s.y,s.heading,s.speed] for n,s in states.items()},
                pairs={n:a['job']['pair_id'] for n,a in active.items()},owners=controller.shared_scheduler.owners))+'\n')
            for name in list(active):
                ex=controller.executions[name];active[name]['maximum_error_m']=max(active[name]['maximum_error_m'],ex.maximum_error)
                if not ex.finished:continue
                job=active[name]['job'];result=dict(pair_id=job['pair_id'],origin=job['origin'],destination=job['destination'],status='PASS',
                    route_sha256=route_fingerprint(plans[name]),ue_physics_validated=True,body_obstacle_contacts=0,vehicle_collisions=0,
                    vehicle=name,duration_s=now-active[name]['start'],maximum_error_m=active[name]['maximum_error_m'],peak_speed_m_s=active[name]['peak_speed_m_s'],
                    terminal_rear_xy_m=[states[name].x,states[name].y],target_rear_xy_m=list(plans[name].point(plans[name].length)))
                results.append(result);stop_monitor(name);fixture('FREEZE',[name])
                clear_collected(controller,name,now,physically_removed=True);del active[name]
                print('PASS',counts()['completed_pairs'],'attempted',len(results),'/',expected,job['pair_id'],flush=True)
            if now-last_status>5:
                report.update(**counts(),active_pairs={n:a['job']['pair_id'] for n,a in active.items()},elapsed_s=now)
                progress_file.write_text(json.dumps(report,indent=2)+'\n');last_status=now
                trajectory.flush()
                print('PROGRESS',len(results),'/',expected,'passed',counts()['completed_pairs'],'issues',counts()['failed_pairs'],'active',len(active),'moving',moving,flush=True)
            time.sleep(.02 if args.time_scale>2 else .04)
        report['controller_events']=controller.events
        passed=counts()['completed_pairs']==expected
        trajectory.close();report.update(**counts(),status='PASS' if passed else 'COMPLETE_WITH_ISSUES',
            elapsed_s=time.monotonic()-begin,ue_physics_tested=True,ue_physics_validated=passed)
    except BaseException as exc:
        report['failure_states']={n:dict(x=s.x,y=s.y,heading=s.heading,speed=s.speed) for n,s in locals().get('states',{}).items()}
        report['failure_audits']=locals().get('audits',{})
        report.update(**counts(),status='FAIL',failure=repr(exc),traceback=traceback.format_exc(),
            active_pairs={n:a['job']['pair_id'] for n,a in active.items()});print('PAIR FAIL',exc,flush=True)
        previous.write_text(json.dumps(report,indent=2)+'\n')
    finally:
        if 'trajectory' in locals():trajectory.close()
        if adapter is not None:adapter.close()
        for name in pool:
            try:stop_monitor(name)
            except Exception:pass
        c.reset();c.simPause(True)
        try:
            fixture('RESTORE')
            # The UE fixture restores the saved world transform directly.
            # After reset(), AirSim can rebuild active cars' local NED origins;
            # replaying pre-reset local poses would move them to the wrong world position.
            stop_dir=ROOT/'runs/ue_scene/test_sites'
            (stop_dir/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=names)))
            ue_script(c,'stop_test_vehicle_motion.py',stop_dir/'motion_stop.json')
        finally:c.simPause(False)
        time.sleep(.5)
        for name,enabled in apis.items():c.enableApiControl(enabled,vehicle_name=name)
        report['fleet_restored']=max(math.dist(home_xy[n],absolute_ue(c,n,origin)[0][:2]) for n in names)<.1
        report['max_restored_speed_m_s']=max(abs(c.getCarState(vehicle_name=n).speed) for n in names)
        report['api_restored']=all(c.isApiControlEnabled(vehicle_name=n)==apis[n] for n in names)
        report['saved_maps_unchanged']=map_fingerprints()==maps
        if not report['fleet_restored'] or not report['api_restored'] or not report['saved_maps_unchanged']:report['status']='FAIL'
        previous.write_text(json.dumps(report,indent=2)+'\n')
        progress_file.write_text(json.dumps(report,indent=2)+'\n')
    print(expected,'NODE PAIRS',report['status'],report['completed_pairs'],flush=True)
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
