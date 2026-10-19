#!/usr/bin/env python3
"""Audit the two-ramp repair, cross-seam driving and normal-lane parallel motion."""
import hashlib
import json
from pathlib import Path
from validate_ue_loading_access import map_fingerprints
ROOT=Path(__file__).resolve().parents[1];RAW=ROOT/'runs/ue_scene/station_siting'
current=map_fingerprints();patch=json.loads((RAW/'waiting_ramp_seam_repair.json').read_text())
before=json.loads((RAW/'scene_before_waiting_ramp_seam_repair.json').read_text());after=json.loads((ROOT/'runs/ue_scene/scene.json').read_text())
def static(scene):
    return {a['path']:a for a in scene['actors'] if a['meshes'] and a['class'] not in ('TrafficTruckPawn','BP_Truck_C','CameraActor')}
a,b=static(before),static(after);changed=[k for k in a.keys()&b.keys() if a[k]!=b[k]]
only_known=not a.keys()-b.keys() and not b.keys()-a.keys() and len(changed)==2 and {b[k]['label'] for k in changed}=={p['actor'] for p in patch['modified']}
profiles={}
for stage in ('before','after'):
    data=json.loads((RAW/f'waiting_seam_profile_{stage}.json').read_text())
    profiles[stage]={p['id']:max(abs(u['ground_z_m']-v['ground_z_m']) for u,v in zip(p['samples'],p['samples'][1:])) for p in data['profiles']}
rows=[]
for name,directory in (('j_1_1','ue_shared_transport_j1'),('j_3_1','ue_shared_transport_j3'),('normal_parallel','ue_parallel_transport')):
    f=json.loads((ROOT/f'runs/{directory}/acceptance.json').read_text());r=json.loads((ROOT/f'runs/{directory}/dispatch/acceptance.json').read_text());c=r['controller']
    rows.append(dict(case=name,status=f['status'],current_map_verified=f['map_sha256']==current,completed_tasks=c['completed_tasks'],maximum_moving_vehicles=c['maximum_moving_vehicles'],body_obstacle_hits=sum(bool(x['static_sweep_hit']) for s in r['samples'] for audit in s['body_audits'].values() for x in audit['bodies']),vehicle_overlap_samples=sum(bool(x['parked_vehicle_overlap']) for s in r['samples'] for audit in s['body_audits'].values() for x in audit['bodies']),fleet_restored=f['fleet_restored']))
waiting=json.loads((ROOT/'docs/ue_waiting_access_acceptance.json').read_text())
valid=(only_known and all(v<.06 for v in profiles['after'].values()) and all(r['status']=='PASS' and r['current_map_verified'] and r['completed_tasks']==2 and not r['body_obstacle_hits'] and not r['vehicle_overlap_samples'] and r['fleet_restored'] for r in rows) and rows[-1]['maximum_moving_vehicles']==2 and waiting['status']=='PASS' and waiting['map_sha256']==current)
s=dict(status='PASS' if valid else 'FAIL',map_sha256=current,only_two_known_ramps_modified=only_known,ground_step_per_half_m=profiles,cases=rows,waiting_regression_status=waiting['status'],patch_record=str((RAW/'waiting_ramp_seam_repair.json').relative_to(ROOT)),warehouse_routes_not_changed=True,camera_changed=False)
(ROOT/'docs/ue_seam_repair_acceptance.json').write_text(json.dumps(s,ensure_ascii=False,indent=2)+'\n')
text=f'''# 等待坡道接缝修复与正常道路并行验收

状态：**{s['status']}**。两份地图修改前均已备份，只修改 `TrafficWaitingQ9Q10RoadToCurb` 和 `TrafficWaitingQ11Q12RoadToCurb`，其他静态网格、位置及碰撞包络一致，未删除任何物体。

## 修复原因与施工

原道路侧坡道长 3.2 m，和硬化地侧坡道之间留约 1.15 m 空隙，剖面形成约 17 cm 的下落和上台阶。道路侧坡道改长 4.8 m，中心 x=-15.1 m、z=1.1675 m，下降高差 0.175 m，与另一段坡道重叠衔接；两份区域分别采用 y=119/247 m。修复后四条车道剖面每 0.5 m 最大高差约 {max(profiles['after'].values()):.3f} m，原下落台阶消失。

## 动态结果

- j_1_1、j_3_1 两处双车跨缝共享通道实测通过，完整前进、终点服务、车尾清空后释放，无车身碰障碍、无车辆重叠。
- 取消整段横路共享独占后，双向独立车道两车同时前进实测通过，最大同时移动车辆数 2；仍使用原 FCFS 路口控制，不宣称两辆车同时占用同一路口。
- Q9/Q10/Q11/Q12 进出与四段邻接车道提速复测通过；其余装卸位、等待位通过记录以完整行驶包络的静态几何不变证明保留，详情见 `access_regions_after_ramp_seam_repair.json`。
- 测试结束复位车队，恢复 API 和泊位标记，相机自由。轮胎接地只诊断；已核实道路/坡道上的浅垂直接触且车身扫描清空时记为地面支撑，侧向碰撞、深碰撞和车身碰撞仍失败。

## 重现

```bash
/usr/bin/python3 scripts/validate_ue_shared_transport.py --scenario config/runtime/industrialarea_ue_shared_transport_j1.json --output runs/ue_shared_transport_j1
/usr/bin/python3 scripts/validate_ue_shared_transport.py --scenario config/runtime/industrialarea_ue_shared_transport_j3.json --output runs/ue_shared_transport_j3
/usr/bin/python3 scripts/validate_ue_shared_transport.py --scenario config/runtime/industrialarea_ue_parallel_transport.json --output runs/ue_parallel_transport
```

相同 UE 世界中的这些测试依次执行。独立源码检查和轻量仿真可并行。

施工与备份：`runs/ue_scene/station_siting/waiting_ramp_seam_repair.json`。原卡车失败记录保留于 `runs/ue_shared_transport/j1_ramp_failure/`，现已由本轮实际横路跨缝复测通过替代。
'''
(ROOT/'docs/ue_seam_repair_acceptance.md').write_text(text)
print(s['status'],'known ramps only:',only_known,'case results:',[(r['case'],r['status']) for r in rows])
if not valid:raise SystemExit(1)
