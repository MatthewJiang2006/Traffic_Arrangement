"""On-demand wheel support evidence for a native ground-contact event."""
import builtins,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
name=builtins.flow_wheel_request
row=builtins.flow_fleet_sample_wheels(name)
(ROOT/'runs/ue_scene'/('flow_wheels_'+name+'.json')).write_text(json.dumps(row))
