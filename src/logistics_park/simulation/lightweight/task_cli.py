import argparse
import json
import shutil
import sys
from dataclasses import replace
from pathlib import Path

from logistics_park.common.logging import configure_logging
from logistics_park.map.map_loader import load_map
from logistics_park.tasks.scenario_generator import cyclic_tasks, fixed_tasks, random_tasks
from logistics_park.tasks.service_station import StationConfig
from .task_simulator import TaskSimulationConfig, TaskSimulator
from .visualization import write_svg


def load_scenario(path: Path, mode: str, seed: int | None = None):
    data = json.loads(path.read_text(encoding="utf-8"))
    stations = [StationConfig(**{**item,"berth_nodes":tuple(item["berth_nodes"])}) for item in data["stations"]]
    homes = data["vehicle_nodes"]
    if mode == "fixed":
        tasks = fixed_tasks(data["fixed"])
    elif mode == "random":
        options = {**data["random"],"destinations":tuple(data["random"]["destinations"])}
        if seed is not None:
            options["seed"] = seed
        tasks = random_tasks(homes,**options)
    else:
        options = {**data["cyclic"],"stops":tuple(data["cyclic"]["stops"])}
        tasks = cyclic_tasks(homes,**options)
    return homes,stations,tasks


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 3 static routing and transport task acceptance")
    parser.add_argument("--map",type=Path,default=Path("config/topology/task_park.json"))
    parser.add_argument("--config",type=Path,default=Path("config/simulation/tasks.json"))
    parser.add_argument("--scenario",type=Path,default=Path("config/tasks/phase_3.json"))
    parser.add_argument("--mode",choices=("fixed","random","cyclic"),default="fixed")
    parser.add_argument("--algorithm",choices=("dijkstra","astar"))
    parser.add_argument("--seed",type=int)
    parser.add_argument("--output",type=Path,default=Path("runs/phase_3"))
    parser.add_argument("--acceptance",action="store_true",help="validate all task modes and both routing algorithms")
    args = parser.parse_args()
    try:
        graph = load_map(args.map)
        config = TaskSimulationConfig.load(args.config)
        configure_logging(args.output/"simulation.log")
        results = []
        algorithms = ("dijkstra","astar") if args.acceptance else [args.algorithm or config.algorithm]
        modes = ("fixed","random","cyclic") if args.acceptance else [args.mode]
        for algorithm in algorithms:
            for mode in modes:
                homes,stations,tasks = load_scenario(args.scenario,mode,args.seed)
                simulator = TaskSimulator(graph,replace(config,algorithm=algorithm),stations,homes,tasks)
                result = simulator.run()
                result["mode"] = mode
                if mode == "random":
                    data = json.loads(args.scenario.read_text(encoding="utf-8"))
                    result["seed"] = args.seed if args.seed is not None else data["random"]["seed"]
                directory = args.output/f"{algorithm}_{mode}" if args.acceptance else args.output
                simulator.write_artifacts(directory,result)
                for source,name in ((args.map,"map.json"),(args.scenario,"scenario.json")):
                    if source.resolve() != (directory/name).resolve():
                        shutil.copyfile(source,directory/name)
                write_svg(graph,simulator.rows,config.vehicle.length,config.vehicle.width,directory/"trajectories.svg",title="Transport task trajectories (metres)")
                results.append(result)
                print(f"{algorithm}/{mode}: {result['status']} | "
                      f"tasks {result['completed_tasks']}/{result['task_count']} | "
                      f"collisions {result['collision_pair_steps']} | "
                      f"capacity violations {result['capacity_violation_station_steps']} | "
                      f"simulation {result['simulation_time']:.2f}s",flush=True)
        passed = all(result["status"].endswith("_PASS") for result in results)
        if args.acceptance:
            # Fixed mode must exercise a full waiting area and a rejected admission.
            fixed = [result for result in results if result["mode"] == "fixed"]
            saturation_pass = all(result["stations"]["W"]["max_reserved"] == 3
                                  and result["stations"]["W"]["max_waiting"] == 2
                                  and result["admission_denial_attempts"] > 0 for result in fixed)
            passed = passed and saturation_pass
            aggregate = {"status":"PHASE_3_TASK_CLOSED_LOOP_PASS" if passed else "PHASE_3_TASK_CLOSED_LOOP_FAIL",
                         "warehouse_saturation_test":"PASS" if saturation_pass else "FAIL","scenarios":results}
            (args.output/"acceptance.json").write_text(json.dumps(aggregate,indent=2)+"\n",encoding="utf-8")
            print(f"{aggregate['status']} | warehouse saturation: {aggregate['warehouse_saturation_test']} | "
                  f"report: {args.output/'acceptance.json'}",flush=True)
        return 0 if passed else 1
    except (ValueError,KeyError,OSError,TypeError) as error:
        parser.exit(2,f"configuration error: {error}\n")
    except KeyboardInterrupt:
        print("\n运输仿真已中断，验收尚未完成。",file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
