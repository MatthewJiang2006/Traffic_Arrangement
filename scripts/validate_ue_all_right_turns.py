#!/usr/bin/env python3
"""Serialized physical acceptance of every legal right turn in the current UE park."""
import hashlib,json,subprocess,sys
from pathlib import Path
from build_ue_turn_clearance import build
ROOT=Path(__file__).resolve().parents[1]
from validate_ue_loading_access import map_fingerprints
b=json.loads((ROOT/'config/topology/industrialarea_ue_transport.json').read_text());turns=[e for e in b['edges'] if e['kind']=='junction' and e.get('maneuver')=='RIGHT']
out=ROOT/'runs/ue_right_turns';rows=[];maps=map_fingerprints()
for i,e in enumerate(turns):
    key=e['edge_id'];folder=out/key;folder.mkdir(parents=True,exist_ok=True)
    print(f"RIGHT TURN {i+1}/{len(turns)} {key}",flush=True)
    sp=build(key)
    if (folder/'acceptance.json').exists() and (folder/'wheel_ground_acceptance.json').exists():
        f=json.loads((folder/'acceptance.json').read_text());g=json.loads((folder/'wheel_ground_acceptance.json').read_text());sc=json.loads((folder/'scenario.json').read_text())
        frozen=json.loads(Path(sc['map']).read_text())
        tested=next(x for x in frozen['edges'] if x['edge_id']==key)
        same=len(tested['centerline'])==len(e['centerline']) and all(__import__('math').dist(a,b)<1e-6 for a,b in zip(tested['centerline'],e['centerline']))
        if f['status']==g['status']=='PASS' and f['map_sha256']==maps and same:
            rows.append(dict(edge=key,status='PASS',evidence=str(folder.relative_to(ROOT)),retained_verified_case=True))
            print(key,'PASS (retained current geometry proof)',flush=True);continue
    with (folder/'validation.log').open('w') as log:
        result=subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/validate_ue_shared_transport.py'),'--scenario',str(sp),'--output',str(folder)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
    status='FAIL'
    if result.returncode==0:
        with (folder/'wheel_ground.log').open('w') as log:
            probe=subprocess.run(['/usr/bin/python3',str(ROOT/'scripts/verify_ue_turn_wheel_surfaces.py'),'--run',str(folder)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        if probe.returncode==0:status='PASS'
    rows.append(dict(edge=key,status=status,evidence=str(folder.relative_to(ROOT))))
    summary=dict(status='PASS' if len(rows)==len(turns) and all(r['status']=='PASS' for r in rows) else 'FAIL' if status=='FAIL' else 'RUNNING',expected_turns=len(turns),cases=rows,map_sha256=maps,topology_sha256=hashlib.sha256((ROOT/'config/topology/industrialarea_ue_transport.json').read_bytes()).hexdigest(),camera_changed=False)
    (out/'acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(key,status,flush=True)
    if status=='FAIL':
        print((folder/'validation.log').read_text()[-1800:],flush=True)
        if (folder/'wheel_ground.log').exists():print((folder/'wheel_ground.log').read_text()[-1000:],flush=True)
        raise SystemExit(1)
print('ALL RIGHT TURNS PASS',flush=True)
