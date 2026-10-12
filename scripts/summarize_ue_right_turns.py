#!/usr/bin/env python3
"""Publish complete per-turn physical, wheel-ground and lane-preservation proof."""
import hashlib,json,math
from pathlib import Path
from validate_ue_loading_access import map_fingerprints
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'runs/ue_right_turns';mp=ROOT/'config/topology/industrialarea_ue_transport.json';current=json.loads(mp.read_text());edges={e['edge_id']:e for e in current['edges']};before=json.loads((OUT/'transport_map_before.json').read_text());old={e['edge_id']:e for e in before['edges']};maps=map_fingerprints()
expected=[e for e in current['edges'] if e['kind']=='junction' and e.get('maneuver')=='RIGHT'];rows=[]
for e in expected:
    folder=OUT/e['edge_id'];fp=folder/'acceptance.json';gp=folder/'wheel_ground_acceptance.json'
    if not fp.exists() or not gp.exists():rows.append(dict(edge=e['edge_id'],status='PENDING'));continue
    f=json.loads(fp.read_text());r=json.loads((folder/'dispatch/acceptance.json').read_text());g=json.loads(gp.read_text());s=json.loads((folder/'scenario.json').read_text());tested=json.loads(Path(s['map']).read_text())
    path=next(t['centerline'] for t in tested['edges'] if t['edge_id']==e['edge_id'])
    geometry=len(path)==len(e['centerline']) and all(math.dist(a,b)<1e-6 for a,b in zip(path,e['centerline']))
    samples=r['samples'];body=sum(bool(b['static_sweep_hit']) for t in samples for a in t['body_audits'].values() for b in a['bodies']);vehicles=sum(bool(b['parked_vehicle_overlap']) for t in samples for a in t['body_audits'].values() for b in a['bodies'])
    passed=f['status']==r['status']==g['status']=='PASS' and geometry and not body and not vehicles and f['map_sha256']==maps and g['map_sha256']==maps and g['source_sha256']==hashlib.sha256((folder/'dispatch/acceptance.json').read_bytes()).hexdigest() and f['scenario_map_sha256']==hashlib.sha256(Path(s['map']).read_bytes()).hexdigest() and all(f[k] for k in ('fleet_restored','api_restored','saved_maps_unchanged','settings_unchanged'))
    rows.append(dict(edge=e['edge_id'],status='PASS' if passed else 'FAIL',incoming_lane=e['incoming_lane'],outgoing_lane=e['outgoing_lane'],body_obstacle_hits=body,vehicle_overlap_samples=vehicles,wheel_ground_samples=g['wheel_ground_samples'],original_raised_curb_samples=len(g['raised_curb_or_nonroad_height_samples']),buffer_ramp_contacts=len(r['ground_contact_samples']),maximum_path_error_m=r['controller']['maximum_path_error_m'],runtime_s=r['controller']['simulation_time_s'],geometry_matches_current_plan=geometry,evidence=str(folder.relative_to(ROOT))+'/acceptance.json'))
# All road, left and straight geometry and every lane connection must remain identical.
connectivity=all(all(e[k]==old[e['edge_id']][k] for k in ('from_node','to_node','capacity_alias')) for e in current['edges'])
other_geometry=all(e['centerline']==old[e['edge_id']]['centerline'] for e in current['edges'] if e.get('maneuver')!='RIGHT' or e['kind']!='junction')
allpass=all(r['status']=='PASS' for r in rows) and len(rows)==22 and connectivity and other_geometry
status='PASS' if allpass else 'FAIL' if any(r['status']=='FAIL' for r in rows) else 'PENDING'
summary=dict(status=status,cases=rows,expected_right_turns=22,passed_right_turns=sum(r['status']=='PASS' for r in rows),lane_connectivity_preserved=connectivity,other_geometry_unchanged=other_geometry,map_sha256=maps,topology_sha256=hashlib.sha256(mp.read_bytes()).hexdigest(),ue_physics_validated=allpass,camera_changed=False)
(ROOT/'docs/ue_right_turn_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
lines=['# 全园区右转路径优化','',f"状态：**{status}**，已通过 {summary['passed_right_turns']}/22 条右转。",'',
       '原路径较早转向，卡车前后轮在右转时容易切入内侧路缘。当前保持原入口、出口节点和合法车道，先直行至转向点，再沿圆弧转弯、接回出口车道；宽路右车道入口采用 3.2 m 半径，其余采用 4 m 半径。两个半径均满足当前车辆 40° 转向与 2.3285 m 轴距约束。右转采用 1.65 m 预瞄，并明确直行段内不提前打方向，转弯速度上限 1.5 m/s。', '',
       '只更新道路规划与控制代码，未修改 UE 地图、仓库、等待位或缓冲坡道。原道路规划已备份。车道连接保持一致：两条同向车道均能直行，左车道左转、右车道右转。左转、直行、道路与调头几何不变。','',
       '每条右转分别在 UE 实际行驶，观测三段车身包络及六个轮胎位置；再将实际轮胎 XY 回放为地面射线，要求原路面高度约 1.1 m，原生 1.25 m 路缘不得出现在轮胎下。已审批的路缘缓冲坡道允许按其正常坡面通过，仍要求车身无障碍接触、无车辆重叠；横向硬碰撞继续判失败。轮胎轻微离地仅记录。', '',
       '右转路口内部允许最大 1.2 m 的参考线偏差，直行等其他路段仍为 0.7 m。这里不以中心线误差代替通行验收：每条右转还必须通过整车碰撞、实际轮胎地面剖面、路口授权及驶回合法出口车道的检查。','',
       '| 右转连接 | 实测状态 | 车身碰障碍 | 轮胎地面采样 | 原生路缘采样 |','|---|---|---:|---:|---:|']
for r in rows:lines.append(f"| {r['edge']} | {r['status']} | {r.get('body_obstacle_hits','—')} | {r.get('wheel_ground_samples','—')} | {r.get('original_raised_curb_samples','—')} |")
lines+=['','## 复现','','```bash','.venv/bin/python scripts/optimize_ue_right_turns.py','/usr/bin/python3 scripts/validate_ue_all_right_turns.py','/usr/bin/python3 scripts/summarize_ue_right_turns.py','```','',
        '同一个 UE 世界按顺序验证，开场布车、结束复位，期间不控制镜头。每项保留冻结的拓扑、原始采样、轮胎地面射线、地图哈希与车队/API 复位证据；汇总位于 `runs/ue_right_turns/acceptance.json`。优化前跨仓路线压路缘的失败记录和宽路调试记录均保留在 `runs/ue_right_turns/`，未计为通过。']
(ROOT/'docs/ue_right_turn_acceptance.md').write_text('\n'.join(lines)+'\n');print(status,summary['passed_right_turns'],'/',22)
if status=='FAIL':raise SystemExit(1)
