#!/usr/bin/env python3
"""Replay measured wheel XY as read-only ground rays to detect raised curb."""
import argparse,hashlib,json
from pathlib import Path
import airsim
from ue_trailer_driving_check import ue_script
from validate_ue_loading_access import map_fingerprints
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);args=p.parse_args()
raw_path=args.run/'dispatch/acceptance.json';raw=json.loads(raw_path.read_text());fixture=json.loads((args.run/'acceptance.json').read_text())
profiles=[]
for i,s in enumerate(raw['samples']):
    for name,a in s['body_audits'].items():
        for w in a['wheels']:
            xy=w['ue_m'][:2];profiles.append(dict(id=f"{i}_{name}_{w['name']}",from_xy_m=xy,to_xy_m=xy))
folder=ROOT/'runs/ue_scene/station_siting';name=args.run.name+'_wheel_ground.json'
(folder/'entry_profile_request.json').write_text(json.dumps(dict(output=name,profiles=profiles)))
r=ue_script(airsim.CarClient(timeout_value=5),'probe_station_entries.py',folder/name)
scenario=json.loads((args.run/'scenario.json').read_text())
ramps=set(scenario.get('verified_ground_actors',[]))
scene=json.loads((ROOT/'runs/ue_scene/scene.json').read_text())
tapers={'TrafficW1LaneTaperLowY','TrafficW1LaneTaperHighY','TrafficW2LaneTaperLowY','TrafficW2LaneTaperHighY'}
ramps.update(a['name'] for a in scene['actors'] if a['name'] in tapers and a['class']=='StaticMeshActor' and a['meshes']==['/Engine/BasicShapes/Cube.Cube'])
bad=[dict(id=e['id'],**e['samples'][0]) for e in r['profiles'] if e['samples'][0]['ground_z_m'] is None or not .99<e['samples'][0]['ground_z_m']<(1.35 if e['samples'][0]['surface_actor'] in ramps else 1.16)]
passed=raw['status']==fixture['status']=='PASS' and fixture['map_sha256']==map_fingerprints() and not bad
summary=dict(status='PASS' if passed else 'FAIL',wheel_ground_samples=len(profiles),raised_curb_or_nonroad_height_samples=bad,map_sha256=map_fingerprints(),source_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),read_only=True)
(args.run/'wheel_ground_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
print(summary['status'],'wheel ground probes',len(profiles),'bad',len(bad))
if not passed:raise SystemExit(1)
