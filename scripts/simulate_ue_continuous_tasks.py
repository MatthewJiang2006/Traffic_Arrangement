#!/usr/bin/env python3
"""Targeted continuous task + waiting validation with bicycle dynamics."""
import json,math,sys,time,hashlib,argparse,functools,pickle
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'src'))
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.transport_control import TransportController
from logistics_park.runtime.task_execution import ServiceRouteGeometry,ContinuousTaskExecutor
from logistics_park.scheduling.fleet_lifecycle import FleetRegistry
from logistics_park.scheduling.task_dispatch import TaskDispatcher,TransportWork,WorkStatus
from logistics_park.common.messages import VehicleState
from logistics_park.scheduling.wait_dependencies import wait_snapshot
from logistics_park.scheduling.traffic_routing import TrafficRouting
from continuous_run_sources import algorithm_fingerprints
from logistics_park.runtime.fleet_progress import FleetProgressWatch,task_progress,LocalProgressWatch,local_progress
from logistics_park.runtime.run_lock import exclusive_run

SCENARIO_PATH=ROOT/'config/runtime/industrialarea_ue_full_verified_20261007.json'
SPEC=json.loads(SCENARIO_PATH.read_text())
POSITIONS=SPEC['positions']
TASKS=SPEC['tasks']

def cache_routes(network):
    """Bounded per-run cache; this runner never mutates the service graph."""
    original=network.route
    @functools.lru_cache(maxsize=2000)
    def cached(origin,destination,options):
        kwargs=dict(options)
        if 'edge_costs' in kwargs:kwargs['edge_costs']=dict(kwargs['edge_costs'])
        return original(origin,destination,**kwargs)
    def route(origin,destination,**kwargs):
        options=tuple(sorted((k,frozenset(v) if k=='occupied_nodes' else tuple(sorted(v.items())) if k=='edge_costs' else v) for k,v in kwargs.items()))
        return cached(origin,destination,options)
    network.route=route


def setup(positions=None,tasks=None):
    positions=POSITIONS if positions is None else positions
    tasks=TASKS if tasks is None else tasks
    path=ROOT/SPEC.get('network','config/topology/industrialarea_ue_service_network.json')
    blocked=set(SPEC['blocked_candidate_edges'])
    network=ServiceNetwork(path,blocked_edges=blocked)
    if len(positions)>2:cache_routes(network)
    model=TransportMap(ROOT/SPEC['map'])
    geometry=ServiceRouteGeometry(network,model);fleet=FleetRegistry()
    traffic=TrafficRouting(network,model,**SPEC.get('traffic_routing_options',{})) if SPEC.get('traffic_routing',False) else None
    dispatch=TaskDispatcher(network,fleet,protect_committed_routes=SPEC.get('rolling_resources',False) or traffic is not None,traffic_routing=traffic,follow_to_busy_destination=SPEC.get('local_traffic',False),park_idle_vehicles=SPEC.get('park_idle_vehicles',False),offroad_busy_wait=SPEC.get('offroad_busy_wait',False),efficiency_policy=SPEC.get('efficiency_policy'),scheduling_options=SPEC.get('scheduling_options'))
    controller=TransportController(model,{},service_s=0,shared_resources=True,collect_at_destination=True,retain_finished_vehicles=True,rolling_resources=SPEC.get('rolling_resources',False),pipeline_shared=SPEC.get('traffic_routing',False),local_traffic=SPEC.get('local_traffic',False))
    executor=ContinuousTaskExecutor(dispatch,geometry,controller);states={}
    for vehicle,node in positions.items():
        site=network.services[node];state=VehicleState(vehicle,*site['rear_xy_m'],math.radians(site['heading_deg']))
        v=fleet.adopt_present_vehicle(vehicle,0,fully_inside=True,resources=[])
        dispatch.place_vehicle(vehicle,v.visit_id,node,0,stopped=True)
        states[vehicle]=state;executor.attach(vehicle,node,state,0)
    for i,(a,b) in enumerate(tasks):dispatch.submit(TransportWork(f'transport-{i}',a,b))
    executor.prepare_schedule(states)
    return executor,states


def run():
    with exclusive_run(ROOT/SPEC.get('output','runs/continuous_tasks')/'lightweight.lock'):
        return _run_locked()


def _run_locked():
    algorithm_at_start=algorithm_fingerprints(ROOT)
    executor,states=setup();d=executor.dispatch;dt=SPEC.get('light_dt_s',.05);begin=time.monotonic();failure=None;peak=0;last_motion=0.
    watch=FleetProgressWatch(SPEC.get('early_stall_check_s',15),SPEC.get('early_stall_displacement_m',.25)) if SPEC.get('early_stall_check_s') else None
    progress_check=None
    local_watch=LocalProgressWatch(SPEC.get('local_stall_check_s',20),SPEC.get('local_wait_limit_s',90)) if SPEC.get('local_traffic') else None
    local_progress_check=None;last_local_check=-1.
    diagnostic_prefix=SPEC.get('diagnostic_preflight_s')
    if diagnostic_prefix is not None and not 60<=diagnostic_prefix<=SPEC.get('light_max_sim_s',2400):
        raise ValueError('Diagnostic preflight must cover at least 60 simulated seconds')
    def checkpoint(now):
        folder=ROOT/SPEC['output'];folder.mkdir(parents=True,exist_ok=True)
        temporary=folder/'checkpoint.tmp'
        with temporary.open('wb') as stream:pickle.dump(dict(executor=executor,states=states,now=now,algorithm_sha256=algorithm_at_start),stream)
        temporary.replace(folder/'checkpoint.pkl')
    try:
        for tick in range(int(SPEC.get("light_max_sim_s",2400)/dt)):
            now=tick*dt;commands=executor.tick(states,now)
            peak=max(peak,sum(abs(s.speed)>.1 for s in states.values()))
            if any(abs(s.speed)>.1 for s in states.values()):last_motion=now
            if tick%max(1,round(SPEC.get('light_status_interval_s',1200*dt)/dt))==0:
                print('LIGHT',round(now),sum(s==WorkStatus.COMPLETE for s in d.status.values()),'tasks',sum(abs(s.speed)>.1 for s in states.values()),'moving',flush=True)
                if SPEC.get('light_checkpoints'):
                    checkpoint(now)
            if watch:
                token,deadline=task_progress(executor,now)
                progress_check=watch.observe(states,now,token,scheduled_activity_until=deadline)
                if progress_check['stalled']:
                    checkpoint(now)
                    raise RuntimeError('Fleet effective progress stalled: '+str(progress_check))
            if local_watch and now-last_local_check>=1.:
                local_progress_check,_=local_progress(executor,states,now,local_watch)
                last_local_check=now
                if local_progress_check['stalled']:
                    checkpoint(now)
                    raise RuntimeError('Local effective progress stalled: '+str(local_progress_check))
            if now-last_motion>SPEC.get('stall_limit_s',2400):raise RuntimeError('Fleet stalled without moving vehicles')
            if diagnostic_prefix is not None and now>=diagnostic_prefix:break
            if d.ready_to_finish():break
            for name,state in states.items():
                target,delta=commands.get(name,(0.,0.));v=state.speed+max(-.6*dt,min(.6*dt,target-state.speed))
                h=state.heading+.5*v*math.tan(delta)/2.3285*dt
                state.x+=v*math.cos(h)*dt;state.y+=v*math.sin(h)*dt
                state.heading+=v*math.tan(delta)/2.3285*dt;state.speed=v;state.timestamp=now
        else:failure='Targeted simulation time limit'
    except Exception as exc:
        failure=repr(exc)
        checkpoint(now)
    completed=sum(s==WorkStatus.COMPLETE for s in d.status.values())
    waiting=[m.destination for m in executor.completed_moves if m.waiting]
    report=dict(status='PASS' if completed==len(TASKS) and d.ready_to_finish() and (waiting or SPEC.get('local_traffic',False)) and not failure else 'FAIL',completed_tasks=completed,expected_tasks=len(TASKS),
        waiting_nodes_visited=waiting,maximum_moving_vehicles=peak,elapsed_sim_s=now,wall_s=time.monotonic()-begin,
        algorithm_sha256=algorithm_at_start,ue_physics_validated=False,maximum_error_m=executor.controller.maximum_error,
        collision_count=executor.controller.collision_count,unauthorized_shared_entry_count=executor.controller.shared_unauthorized_count,
        shared_owners=dict(executor.controller.shared_scheduler.owners) if executor.controller.shared_scheduler else {},shared_queues={k:list(executor.controller.shared_scheduler.queued(k)) for k in executor.controller.shared_scheduler.channels} if executor.controller.shared_scheduler else {},
        source_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (SCENARIO_PATH,ROOT/SPEC['map'],ROOT/SPEC.get('network','config/topology/industrialarea_ue_service_network.json'))},failure=failure,dispatch_events=d.events,road_events=executor.controller.events,
        wait_dependencies=wait_snapshot(executor,states),
        local_denials=executor.controller.local_traffic.last_denials if executor.controller.local_traffic else {},
        passage_denials=executor.controller.local_traffic.passage_denials if executor.controller.local_traffic else {},
        local_unauthorized_entry_count=executor.controller.local_unauthorized_count,
        road_capacity_violation_steps=executor.controller.capacity_violation_steps,
        timing_statistics=executor.controller.local_traffic.timing.statistics if executor.controller.local_traffic else {},
        timing_events=executor.controller.local_traffic.timing.events if executor.controller.local_traffic else [],
        local_events=executor.controller.local_traffic.events if executor.controller.local_traffic else [],
        progress_check=progress_check,local_progress_check=local_progress_check,
        remaining_assignments={v:dict(task=a.work.task_id,state=a.state,target=d._target(a)) for v,a in d.assignments.items()},
        positions={v:[s.x,s.y,s.heading,s.speed] for v,s in states.items()},node_holders=d.holders,park_idle_vehicles=d.park_idle_vehicles,parked_nodes={v:n for v,n in d.positions.items() if d.network.services[n]['kind']=='waiting'},parking_complete=d.ready_to_finish())
    out=ROOT/SPEC.get('output','runs/continuous_tasks');out.mkdir(parents=True,exist_ok=True)
    if diagnostic_prefix is not None and now>=diagnostic_prefix and not failure:
        report.update(status='DIAGNOSTIC_READY',diagnostic_preflight_s=diagnostic_prefix,vehicle_count=len(states))
    (out/'lightweight_acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],completed,'/',len(TASKS),'waiting',waiting,'failure',failure,flush=True)
    return 0 if report['status'] in ('PASS','DIAGNOSTIC_READY') else 1

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--scenario',default=str(SCENARIO_PATH));args=parser.parse_args()
    SCENARIO_PATH=Path(args.scenario).resolve();SPEC=json.loads(SCENARIO_PATH.read_text());POSITIONS=SPEC['positions'];TASKS=SPEC['tasks']
    raise SystemExit(run())
