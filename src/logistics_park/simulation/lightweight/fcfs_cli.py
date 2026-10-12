import argparse
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import sys

from logistics_park.common.logging import configure_logging
from logistics_park.common.messages import Task
from logistics_park.map.map_loader import load_map
from logistics_park.tasks.service_station import StationConfig
from .fcfs_simulator import FCFSTrafficSimulator, IntersectionControl, TrafficRules
from .task_simulator import TaskSimulationConfig
from .kinematics import VehicleLimits
from logistics_park.navigation.pure_pursuit import TrackingConfig
from .visualization import write_svg
from .replay import write_replay


def load_case(config_path: Path, name: str):
    data=json.loads(config_path.read_text(encoding='utf-8'))
    item=next((case for case in data['cases'] if case['name']==name),None)
    if item is None:
        raise ValueError('unknown FCFS test case')
    # Manifest paths are relative to this config file, independent of cwd.
    root=config_path.resolve().parent
    map_path=root/item['map']
    scenario_path=root/item['scenario']
    scenario=json.loads(scenario_path.read_text(encoding='utf-8'))
    settings=data['simulation'].copy()
    settings['vehicle']=VehicleLimits(**settings.get('vehicle',{}))
    settings['tracking']=TrackingConfig(**settings.get('tracking',{}))
    config=TaskSimulationConfig(**settings)
    rules=TrafficRules(**data['rules'])
    graph=load_map(map_path)
    stations=[StationConfig(**{**station,'berth_nodes':tuple(station['berth_nodes'])}) for station in scenario['stations']]
    specs=[IntersectionControl(**{**spec,'bounds':tuple(spec['bounds'])}) for spec in scenario['intersections']]
    simulator=FCFSTrafficSimulator(graph,config,stations,scenario['vehicle_nodes'],
                                  [Task(**task) for task in scenario['tasks']],specs,rules,scenario.get('initial_edges'))
    return simulator,scenario,map_path,scenario_path


def acceptance_checks(simulator,scenario,result):
    checks={'all_tasks_and_safety':result['status'].endswith('_PASS')}
    required=scenario.get('checks',{})
    if required.get('parallel_motion'):
        checks['parallel_motion']=result['max_translating_vehicles']>=2
    grants=[event for event in simulator.scheduler.events if event['event']=='GRANTED' and event['intersection_id']=='cross']
    if required.get('same_lane_fifo'):
        checks['same_lane_fifo']=[event['vehicle_id'] for event in grants]==['car_02','car_01','car_00']
    if required.get('independent_junctions'):
        checks['independent_junctions']=result['max_simultaneous_intersection_owners']>=2
    if required.get('downstream_full'):
        first=next((event for event in grants if event['vehicle_id']=='car_00'),None)
        checks['downstream_full']=bool(first and first['time']>=8 and result['capacity_blocked_attempts']>0)
    return checks


def main():
    parser=argparse.ArgumentParser(description='Phase 4 concurrent FCFS verification on temporary test maps')
    parser.add_argument('--config',type=Path,default=Path('config/simulation/fcfs.json'))
    parser.add_argument('--case',default='four_way')
    parser.add_argument('--acceptance',action='store_true')
    parser.add_argument('--output',type=Path,default=Path('runs/phase_4'))
    args=parser.parse_args()
    try:
        configure_logging(args.output/'simulation.log')
        data=json.loads(args.config.read_text(encoding='utf-8'))
        names=[case['name'] for case in data['cases']] if args.acceptance else [args.case]
        results=[]
        for name in names:
            sim,scenario,map_path,scenario_path=load_case(args.config,name)
            result=sim.run()
            result['case']=name
            result['acceptance_checks']=acceptance_checks(sim,scenario,result)
            if not all(result['acceptance_checks'].values()):
                result['status']='PHASE_4_FCFS_CLOSED_LOOP_FAIL'
            directory=args.output/name if args.acceptance else args.output
            sim.write_artifacts(directory,result)
            (directory/'traffic_rules.json').write_text(json.dumps(asdict(sim.rules),indent=2)+'\n')
            for source,target in ((map_path,'map.json'),(scenario_path,'scenario.json')):
                if source.resolve()!=(directory/target).resolve():
                    shutil.copyfile(source,directory/target)
            write_svg(sim.graph,sim.rows,sim.config.vehicle.length,sim.config.vehicle.width,
                      directory/'trajectories.svg',title=f'FCFS verification: {name}')
            write_replay(sim,directory/'replay.html')
            results.append(result)
            print(f"{name}: {result['status']} | tasks {result['completed_tasks']}/{result['task_count']} | "
                  f"parallel vehicles {result['max_translating_vehicles']} | mean junction wait {result['mean_junction_waiting_time_s']:.2f}s",flush=True)
        passed=all(result['status'].endswith('_PASS') for result in results)
        if args.acceptance:
            aggregate={'status':'PHASE_4_FCFS_CLOSED_LOOP_PASS' if passed else 'PHASE_4_FCFS_CLOSED_LOOP_FAIL','scenarios':results}
            (args.output/'acceptance.json').write_text(json.dumps(aggregate,indent=2)+'\n')
            print(f"{aggregate['status']} | report: {args.output/'acceptance.json'}",flush=True)
        return 0 if passed else 1
    except (ValueError,KeyError,OSError,TypeError) as error:
        parser.exit(2,f'configuration error: {error}\n')
    except KeyboardInterrupt:
        print('\nFCFS 验证已中断，验收尚未完成。',file=sys.stderr)
        return 130


if __name__=='__main__':
    raise SystemExit(main())
