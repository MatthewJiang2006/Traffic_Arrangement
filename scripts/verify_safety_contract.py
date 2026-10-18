#!/usr/bin/env python3
"""Print safety proof gaps without starting UE or writing experimental data.

Exit 2 means no safety certificate is established; exit 1 is a tool error.
"""
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from logistics_park.safety.verification import assess_contract, physical_command_ttl


def main():
    spec = json.loads((ROOT / 'config/runtime/industrialarea_ue_full_verified_20261007.json').read_text())
    model = json.loads((ROOT / spec['map']).read_text())
    ttl = physical_command_ttl((ROOT / 'scripts/validate_ue_continuous_tasks.py').read_text())
    print(json.dumps(assess_contract(model, ttl), ensure_ascii=False, indent=2))
    return 2


if __name__ == '__main__':
    raise SystemExit(main())
