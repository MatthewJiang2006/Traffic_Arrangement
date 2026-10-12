#!/usr/bin/env python3
"""Run the project checks with third-party pytest autoload disabled."""
import os
from pathlib import Path
import subprocess
import sys

def main() -> int:
    root = Path(__file__).resolve().parents[1]
    env = os.environ.copy()
    env["PYTEST_DISABLE_PLUGIN_AUTOLOAD"] = "1"
    try:
        print("[1/4] Running pytest...", flush=True)
        result = subprocess.run([sys.executable, "-m", "pytest", "-q"], cwd=root, env=env)
        if result.returncode:
            return result.returncode
        print("[2/4] Running 1–4 vehicle acceptance...", flush=True)
        result = subprocess.run(
            [sys.executable, "-m", "logistics_park.simulation.lightweight.cli", "--acceptance",
             "--output", "runs/phase_0_2"], cwd=root, env=env)
        if result.returncode:
            return result.returncode
        print("[3/4] Running Phase 3 fixed/random/cyclic transport acceptance...", flush=True)
        result = subprocess.run(
            [sys.executable, "-m", "logistics_park.simulation.lightweight.task_cli", "--acceptance",
             "--output", "runs/phase_3"], cwd=root, env=env)
        if result.returncode:
            return result.returncode
        print("[4/4] Running concurrent FCFS and downstream capacity acceptance...", flush=True)
        return subprocess.run(
            [sys.executable, "-m", "logistics_park.simulation.lightweight.fcfs_cli", "--acceptance",
             "--output", "runs/phase_4"], cwd=root, env=env).returncode
    except KeyboardInterrupt:
        print("\n检查已中断，尚未完成验收。重新运行此命令即可。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
