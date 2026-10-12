import argparse
import json
import shutil
from pathlib import Path

from logistics_park.common.logging import configure_logging
from logistics_park.map.map_loader import load_map
from .simulator import SimulationConfig, Simulator
from .visualization import write_svg


def main() -> int:
    parser = argparse.ArgumentParser(description="Phase 0-2 minimal lightweight acceptance")
    parser.add_argument("--map", type=Path, default=Path("config/topology/minimal_loop.json"))
    parser.add_argument("--config", type=Path, default=Path("config/simulation/minimal.json"))
    parser.add_argument("--vehicles", type=int, choices=range(1,5), default=4)
    parser.add_argument("--output", type=Path, default=Path("runs/minimal"))
    parser.add_argument("--acceptance", action="store_true", help="validate 1, 2, 3 and 4 vehicles")
    args = parser.parse_args()
    try:
        graph = load_map(args.map)
        config = SimulationConfig.load(args.config)
        args.output.mkdir(parents=True, exist_ok=True)
        configure_logging(args.output/"simulation.log")
        results = []
        for count in (range(1,5) if args.acceptance else [args.vehicles]):
            simulator = Simulator(graph, config, count)
            result = simulator.run()
            directory = args.output/f"vehicles_{count}" if args.acceptance else args.output
            simulator.write_artifacts(directory, result)
            # Store the exact input topology alongside the effective configuration.
            if args.map.resolve() != (directory/"map.json").resolve():
                shutil.copyfile(args.map, directory/"map.json")
            write_svg(graph, simulator.rows, config.vehicle.length, config.vehicle.width, directory/"trajectories.svg")
            results.append(result)
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
        passed = all(result["status"].endswith("_PASS") for result in results)
        if args.acceptance:
            aggregate = {"status": "LIGHTWEIGHT_SIM_MINIMAL_CLOSED_LOOP_PASS" if passed else "LIGHTWEIGHT_SIM_MINIMAL_CLOSED_LOOP_FAIL", "scenarios": results}
            (args.output/"acceptance.json").write_text(json.dumps(aggregate, indent=2)+"\n", encoding="utf-8")
        return 0 if passed else 1
    except (ValueError, KeyError, OSError, TypeError) as error:
        parser.exit(2, f"configuration error: {error}\n")


if __name__ == "__main__":
    raise SystemExit(main())
