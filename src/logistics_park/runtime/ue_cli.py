"""Run configured UE traffic through the project AirSim Adapter."""
import argparse
import signal
import json
from pathlib import Path
from logistics_park.adapters.airsim import AirSimAdapter,AirSimConfig
from logistics_park.map.map_loader import load_map
from .body_observer import FileBodyObserver
from .ue_dispatch import run_dispatch
from .ue_transport import run_transport


def main():
    def terminate(signum, frame):raise SystemExit(128+signum)
    signal.signal(signal.SIGTERM,terminate)
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario',type=Path,default=Path('config/runtime/industrialarea_ue_four_truck.json'))
    parser.add_argument('--output',type=Path,default=Path('runs/airsim_dispatch'))
    parser.add_argument('--body-directory',type=Path,default=Path('runs/ue_scene'))
    args=parser.parse_args();scenario=json.loads(args.scenario.read_text(encoding='utf-8'))
    cfg=dict(scenario['adapter']);cfg['vehicle_ids']=tuple(cfg['vehicle_ids']);cfg['origin_ue_m']=tuple(cfg['origin_ue_m'])
    adapter=AirSimAdapter(AirSimConfig(**cfg),args.output/'adapter.jsonl')
    try:
        adapter.open()
        if scenario.get('application')=='shared_transport':
            report=run_transport(adapter,scenario,FileBodyObserver(args.body_directory,
                                prefix=scenario.get('body_prefix','shared'),wheel_diagnostic_only=True),args.output)
        else:
            report=run_dispatch(adapter,load_map(Path(scenario['map'])),json.loads(Path(scenario['survey']).read_text(encoding='utf-8')),
                            scenario,FileBodyObserver(args.body_directory),args.output)
        print('AirSim project dispatch:',report['status'])
        return 0 if report['status']=='PASS' else 1
    finally:adapter.close()


if __name__=='__main__':raise SystemExit(main())
