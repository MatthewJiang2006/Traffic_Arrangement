#!/usr/bin/env python3
"""Publish actual ordered-pair evidence separately from kinematic prechecks."""
import hashlib,json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
folder=ROOT/'runs/ue_node_pairs'
scenario=json.loads((ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').read_text())
evidence=max((p for p in (folder/'acceptance.json',folder/'progress.json')
              if p.exists() and json.loads(p.read_text()).get('expected_pairs')==132),key=lambda p:p.stat().st_mtime_ns)
report=json.loads(evidence.read_text())
edge_specs={e['edge_id']:e for e in json.loads((ROOT/scenario['map']).read_text())['edges']}
nodes=scenario['node_ids'];rows={r['pair_id']:r for r in report['results']}
for r in rows.values():
    if r['status']=='FAIL':
        edge=edge_specs[r['edge_id']]
        r['maneuver']=edge.get('maneuver')
        r['source_road_edge']=edge.get('source_road_edge',r['edge_id'].split('__test')[0])
expected={j['pair_id'] for j in scenario['jobs']}
passed={k for k,r in rows.items() if r['status']=='PASS'}
verified=(report['status']=='PASS' and passed==expected and report.get('ue_physics_validated')
          and report.get('fleet_restored') and report.get('api_restored') and report.get('saved_maps_unchanged')
          and report.get('body_obstacle_contacts')==0 and report.get('vehicle_collisions')==0)
if report.get('topology_sha256')!=hashlib.sha256((ROOT/scenario['map']).read_bytes()).hexdigest():
    raise RuntimeError('Evidence refers to a different route topology')
publication=dict(status='PASS' if verified else report['status'],expected_pairs=132,completed_pairs=len(passed),
    attempted_pairs=len(rows),failed_pairs=sum(r['status']=='FAIL' for r in rows.values()),
    ue_physics_validated=bool(verified),node_ids=nodes,results=list(rows.values()),evidence_file=str(evidence.relative_to(ROOT)),
    topology_sha256=report['topology_sha256'],map_sha256=report['map_sha256'],
    maximum_moving_vehicles=report['maximum_moving_vehicles'],elapsed_s=report.get('elapsed_s'),
    body_obstacle_contacts=report['body_obstacle_contacts'],vehicle_collisions=report['vehicle_collisions'])
(ROOT/'docs/ue_node_pair_acceptance.json').write_text(json.dumps(publication,indent=2)+'\n')
if verified:
    (ROOT/'config/planning/ue_node_pair_acceptance.json').write_text(json.dumps(publication,indent=2)+'\n')
labels={n:f"W{n[1]}{'L' if n.endswith('loading') and not n.endswith('unloading') else 'U'}" for n in nodes}
lines=['# 装卸节点之间的 132 条有向路径验收','',
    f"UE 状态：**{publication['status']}**；已测 **{len(rows)}/132**，通过 **{len(passed)}**，问题 **{publication['failed_pairs']}**。",'',
    'L 表示装货区，U 表示卸货区；行是起点，列是终点。等待区不作为测试起终点。✓ 为 UE 已完成并停稳到达，✗ 为发现问题，· 为待测，— 为同一节点无需测试。','',
    '| 起点 \\ 终点 | '+' | '.join(labels[n] for n in nodes)+' |',
    '| --- | '+' | '.join('---' for _ in nodes)+' |']
for a in nodes:
    lines.append('| '+labels[a]+' | '+' | '.join('—' if a==b else '✓' if a+'__'+b in passed else '✗' if a+'__'+b in rows else '·' for b in nodes)+' |')
issues=[r for r in rows.values() if r['status']=='FAIL']
if issues:
    lines+=['','## 待集中修复的问题','','转向根据轨迹的 maneuver 字段判定；路段名中的 right 可能是车道名。','',
            '| 路径 | 动作 | 出错路段 | 位置 / m | 原因 |','| --- | --- | --- | --- | --- |']
    for r in issues:
        state=r.get('state') or {};xy=f"({state.get('x',0):.2f}, {state.get('y',0):.2f})"
        action={'LEFT':'左转','RIGHT':'右转','STRAIGHT':'直行','UTURN':'掉头'}.get(r.get('maneuver'),r.get('maneuver',''))
        lines.append(f"| {r['pair_id']} | {action} | {r.get('source_road_edge','')} | {xy} | {r.get('reason','').replace('|','/')} |")
lines+=['',f"最多同时行驶：{report['maximum_moving_vehicles']} 辆。实际耗时：{report.get('elapsed_s',0):.1f} 秒。",'',
    '路线按空闲车辆滚动接续，剩余冲突区域相互排斥；完成后临时隐藏并禁用碰撞，结束时恢复位置、可见性、碰撞、物理、API 和场景时间流速。相机不锁定。', '',
    '轻量预检另存于 `runs/ue_node_pairs/lightweight_acceptance.json`，不能替代 UE 物理验收。', '',
    f"原始证据：`{evidence.relative_to(ROOT)}`。正式发布到 `config/planning/` 仅限完整 132 条 UE 路径通过且场景恢复核对通过。"]
(ROOT/'docs/ue_node_pair_acceptance.md').write_text('\n'.join(lines)+'\n')
print(publication['status'],'attempted',len(rows),'/132','passed',len(passed),'issues',publication['failed_pairs'])
