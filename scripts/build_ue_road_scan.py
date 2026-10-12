#!/usr/bin/env python3
"""Select a compact concurrent physical scan of the node-pair road network."""
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.route_coverage import select_scan
import hashlib


def main():
    source=ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json'
    scenario=json.loads(source.read_text())
    map_path=ROOT/scenario['map']
    plan=select_scan(json.loads(map_path.read_text()),scenario)
    plan.update(source_scenario=str(source.relative_to(ROOT)),
                scenario_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                topology_sha256=hashlib.sha256(map_path.read_bytes()).hexdigest())
    output=ROOT/'config/runtime/industrialarea_ue_road_scan.json'
    output.write_text(json.dumps(plan,indent=2)+'\n')
    print(json.dumps({k:v for k,v in plan.items() if k not in
        ('requirements_by_pair','regions_by_pair','selected_pair_ids','parallel_waves')},indent=2))


if __name__=='__main__':main()
