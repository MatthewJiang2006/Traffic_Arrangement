#!/usr/bin/env python3
"""Audit all native-frame truck polygons against the exported loaded pavement."""
import argparse
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.drivable_surface import DrivableSurface
from logistics_park.runtime.drivable_audit import audit_frames


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();surface=DrivableSurface(json.loads((args.run/'drivable_surface.json').read_text()))
    with (args.run/'native_road_history.jsonl').open() as stream:
        result=audit_frames((json.loads(line) for line in stream),surface)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='flags'}))
    return 0 if result['status']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
