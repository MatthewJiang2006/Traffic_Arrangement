#!/usr/bin/env python3
"""Fast rolling multi-car precheck of all 132 node-pair paths."""
import argparse,hashlib,json,math,sys,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
from logistics_park.runtime.transport_control import TransportController,envelope
from logistics_park.runtime.node_pair_validation import can_stage,clear_collected
from logistics_park.common.messages import VehicleState


def plans_for(model,jobs):
    return {j['pair_id']:RouteGeometry(model,j['edges'],j['checkpoints'],j['origin'],j.get('stops',[j['destination']])) for j in jobs}


def run():
    parser=argparse.ArgumentParser()
    modes=parser.add_mutually_exclusive_group()
    modes.add_argument('--road-scan',action='store_true')
    modes.add_argument('--waiting-access',action='store_true')
    parser.add_argument('--scenario',type=Path,help='Custom local waiting scan manifest')
    parser.add_argument('--output-dir',type=Path,help='Isolated local scan evidence directory')
    args=parser.parse_args()
    if (args.scenario or args.output_dir) and not args.waiting_access:raise RuntimeError('Custom paths require --waiting-access')
    folder=(args.output_dir or ROOT/'runs/ue_node_pairs').resolve();folder.mkdir(parents=True,exist_ok=True)

    scenario_path=args.scenario or ROOT/('config/runtime/industrialarea_ue_waiting_scan.json' if args.waiting_access else 'config/runtime/industrialarea_ue_node_pair_tests.json')
    spec=json.loads(scenario_path.read_text())
    scenario_hash=hashlib.sha256(scenario_path.read_bytes()).hexdigest()
    source_hash=hashlib.sha256((ROOT/spec['map']).read_bytes()).hexdigest()
    prefix='waiting_' if args.waiting_access else 'scan_' if args.road_scan else ''
    if args.road_scan:
        scan=json.loads((ROOT/'config/runtime/industrialarea_ue_road_scan.json').read_text())
        if scan['topology_sha256']!=source_hash or scan['scenario_sha256']!=hashlib.sha256(scenario_path.read_bytes()).hexdigest():
            raise RuntimeError('Road scan sources have changed; rebuild the scan')
        chosen=set(scan['selected_pair_ids'])
        spec['jobs']=[j for j in spec['jobs'] if j['pair_id'] in chosen]
    expected=len(spec['jobs'])
    model=TransportMap(ROOT/spec['map']);templates=plans_for(model,spec['jobs'])
    pending=sorted(spec['jobs'],key=lambda j:-j['distance_m']);slots=[f'Car_{i//3}_{i%3}' for i in range(12)]
    plans={};controller=TransportController(model,plans,service_s=spec.get('service_s',.3),shared_resources=True,collect_at_destination=True)
    states={};assigned={};results=[];dt=.1;begin=time.monotonic();maximum_moving=0;failure=[]
    for tick in range(240000):
        now=tick*dt
        if tick%1000==0:print('LIGHT TIME',now,'done',len(results),'active',len(states),'owners',controller.shared_scheduler.owners,'moving',sum(abs(s.speed)>.1 for s in states.values()),flush=True)
        if tick%1000==0 and states:
            (folder/(prefix+'lightweight_progress.json')).write_text(json.dumps({n:dict(pair=assigned[n]['pair_id'],progress=ex.progress,speed=states[n].speed,visits=[dict(channel=v.channel_id,start=v.start,end=v.end,entered=i in ex.shared_entered,cleared=i in ex.shared_cleared) for i,v in enumerate(ex.plan.shared_visits)]) for n,ex in controller.executions.items()},indent=2))
        if tick%10==0:
            for name in slots:
                if name in states:continue
                selected=next((j for j in pending if can_stage(controller,templates[j['pair_id']])),None)
                if selected is None:continue
                pending.remove(selected);p=templates[selected['pair_id']];plans[name]=p;assigned[name]=selected
                states[name]=VehicleState(name,*p.point(0),p.heading(0));controller.activate(name,now)
                controller.boxes[name]=[envelope(states[name])]
        if not states:
            if pending:
                if tick%10:continue
                failure.append('No test origin can be staged')
            break
        try:commands=controller.tick(states,now)
        except RuntimeError as exc:
            failure.append(str(exc))
            print('DEBUG',[(n,assigned[n]['pair_id'],round(controller.executions[n].progress,3),round(s.x,3),round(s.y,3),round(s.heading,3),plans[n].edges[plans[n].edge_index(controller.executions[n].progress)]) for n,s in states.items()],flush=True)
            break
        maximum_moving=max(maximum_moving,sum(abs(s.speed)>.1 for s in states.values()))
        for name in list(states):
            if controller.executions[name].finished:
                result=dict(pair_id=assigned[name]['pair_id'],status='PASS',maximum_error_m=controller.executions[name].maximum_error)
                results.append(result);clear_collected(controller,name,now,physically_removed=True);del states[name]
                (folder/(prefix+'lightweight_completed.json')).write_text(json.dumps(results,indent=2)+'\n')
                if len(results)%12==0:print('LIGHT PAIRS',len(results),'/',expected,'moving peak',maximum_moving,flush=True)
        for name,state in states.items():
            target,delta=commands[name];v=state.speed+max(-.6*dt,min(.6*dt,target-state.speed))
            h=state.heading+.5*v*math.tan(delta)/2.3285*dt
            state.x+=v*math.cos(h)*dt;state.y+=v*math.sin(h)*dt
            state.heading+=v*math.tan(delta)/2.3285*dt;state.speed=v;state.timestamp=now
        if not states and not pending:break
    else:failure.append('Simulation time limit')
    report=dict(status='PASS' if len(results)==expected and not failure else 'FAIL',expected_pairs=expected,completed_pairs=len(results),
        results=results,failure=failure,maximum_moving_vehicles=maximum_moving,
        topology_sha256=source_hash,scenario_sha256=scenario_hash,step_s=dt,
        simulated_time_s=now,wall_time_s=time.monotonic()-begin,ue_physics_validated=False)
    if args.road_scan:report.update(scope=scan['scope'],end_to_end_132_physically_validated=False)
    (folder/(prefix+'lightweight_acceptance.json')).write_text(json.dumps(report,indent=2)+'\n')
    print(report['status'],len(results),'pairs',failure,'wall',report['wall_time_s'],flush=True)
    return 0 if report['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(run())
