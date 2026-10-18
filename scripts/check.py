#!/usr/bin/env python3
"""Check the current project; optionally run its full lightweight precheck."""
import argparse
import os
from pathlib import Path
import subprocess
import sys


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--precheck', action='store_true', help='Also run the current 32-task lightweight precheck')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1')
    env['PYTHONPATH'] = str(root/'src') + os.pathsep + env.get('PYTHONPATH', '')
    commands = [[sys.executable, '-m', 'pytest', '-p', 'no:nose', '-q', 'tests']]
    if args.precheck:
        commands.append([sys.executable, str(root/'scripts/simulate_ue_continuous_tasks.py')])
    try:
        for command in commands:
            result = subprocess.run(command, cwd=root, env=env)
            if result.returncode:
                return result.returncode
        return 0
    except KeyboardInterrupt:
        print('检查已中断，尚未完成。', file=sys.stderr)
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
