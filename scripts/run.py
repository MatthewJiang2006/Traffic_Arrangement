#!/usr/bin/env python3
"""Run the verified full UE demo with fixed reproduction parameters."""

import argparse
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / 'config/runtime/industrialarea_ue_full_verified_20261007.json'


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()
    stages = (
        ('完整轻量预检', 'simulate_ue_continuous_tasks.py', ()),
        ('完整 UE 验收（1 倍速）', 'validate_ue_continuous_tasks.py', ('--time-scale', '1')),
    )
    for label, script, extra_args in stages:
        print(label, flush=True)
        result = subprocess.run(
            [sys.executable, str(ROOT / 'scripts' / script),
             '--scenario', str(SCENARIO), *extra_args],
            cwd=ROOT,
        )
        if result.returncode:
            print(f'{label}失败，停止运行。', file=sys.stderr, flush=True)
            return result.returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
