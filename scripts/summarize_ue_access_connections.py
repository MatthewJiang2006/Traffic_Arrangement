#!/usr/bin/env python3
"""Publish only exact access geometry that completed restored UE trials."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'runs/ue_access_connections'


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    candidates=json.loads((ROOT/'config/planning/ue_access_connection_candidates.json').read_text())
    preservation=json.loads((OUT/'wall_clearance_preservation.json').read_text())
    static=json.loads((OUT/'static_precheck.json').read_text())
    if preservation['status']!='PASS' or static['status']!='PASS':raise RuntimeError('Prechecks incomplete')
    rows=[];connections={};reverse_sites={}
    port_groups={1:['port_2','port_6','port_64','port_65'],2:['port_62','port_63','port_61'],3:['port_60','port_78']}
    reverse_groups={1:['W3-loading','Q9'],2:['W4-unloading'],3:['Q3']}
    for batch in candidates['batches']:
        number=batch['batch'];folder=OUT/f'batch_{number}'
        fixture=json.loads((folder/'acceptance.json').read_text());run=json.loads((folder/'dispatch/acceptance.json').read_text())
        scenario=json.loads((folder/'scenario.json').read_text())
        current=json.loads((ROOT/batch['scenario']).read_text())
        if scenario['routes']!=current['routes']:raise RuntimeError('Scenario routes changed')
        if digest(folder/'topology.json')!=digest(ROOT/current['map']):raise RuntimeError('Tested geometry changed')
        required=['fleet_restored','api_restored','idle_visibility_restored','settings_unchanged','saved_maps_unchanged']
        if fixture['status']!='PASS' or run['status']!='PASS' or not all(fixture[k] for k in required):
            raise RuntimeError('Batch not accepted/restored '+str(number))
        if fixture['map_sha256']!=preservation['after_map_sha256']:raise RuntimeError('Different saved map')
        controller=run['controller'];samples=run['samples']
        hits=sum(bool(body['static_sweep_hit']) for sample in samples for audit in sample['body_audits'].values() for body in audit['bodies'])
        overlaps=sum(bool(body['parked_vehicle_overlap']) for sample in samples for audit in sample['body_audits'].values() for body in audit['bodies'])
        if hits or overlaps or batch['lightweight']['status']!='PASS':raise RuntimeError('Collision or lightweight failure')
        evidence=str((folder/'acceptance.json').relative_to(ROOT))
        row=dict(batch=number,status='PASS',cases=batch['cases'],vehicles=controller['vehicle_count'],
            maximum_simultaneously_moving=controller['maximum_moving_vehicles'],
            runtime_s=controller['simulation_time_s'],maximum_path_error_m=controller['maximum_path_error_m'],
            body_hits=hits,vehicle_overlaps=overlaps,fleet_restored=True,camera_changed=False,evidence=evidence)
        rows.append(row)
        for pid in port_groups[number]:
            c=dict(candidates['connections'][pid]);c.update(status='PASS',evidence=evidence,
                topology_sha256=digest(folder/'topology.json'),fixture_sha256=digest(folder/'acceptance.json'))
            connections[pid]=c
        for site in reverse_groups[number]:reverse_sites[site]=dict(status='PASS',evidence=evidence)
    report=dict(schema_version=1,status='PASS',connections=connections,verified_reverse_sites=reverse_sites,
        map_sha256=preservation['after_map_sha256'],preserved_source_map_sha256=preservation['before_map_sha256'],
        static_actor_preservation=preservation,batches=rows,routing_enabled=False,
        scope='Nine boundary connections and four local reverse accesses; not 552 complete physical transports',
        source_candidates_sha256=digest(ROOT/'config/planning/ue_access_connection_candidates.json'))
    (ROOT/'config/planning/ue_access_connection_acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    (ROOT/'docs/ue_access_connections_acceptance.json').write_text(json.dumps(report,indent=2)+'\n')
    lines=['# 车道接入与倒车方案验收','',
        '9 个边界端口均采用连续并入轨迹，4 个新增倒车方案均已通过低速 UE 验证。分成 5 车、3 车、单车三组；同组路线空间不冲突，结束恢复 24 车姿态/API/外观，镜头自由。','',
        '| 组 | 项目 | 同时行驶峰值 | 用时 s | 最大偏差 m |','|---|---|---:|---:|---:|']
    for row in rows:lines.append(f"| {row['batch']} | {', '.join(row['cases'])} | {row['maximum_simultaneously_moving']} | {row['runtime_s']:.1f} | {row['maximum_path_error_m']:.3f} |")
    lines+=['',
        '首次五车试验在 Q5 出位时碰到 `SM_Block_Wall_68`，已制动复位，失败记录保留于 `runs/ue_access_connections/batch_1_before_wall_clearance/`。Q3 静态检查还发现 `SM_Block_Wall_57` 干涉。按用户已有小围挡拆除授权，备份地图后拆除这两段约 3.4 m 长的小围挡；编辑世界和当前 PIE 同步移除并保存，不重启仿真。前后场景核对确认其他静态物体、变换和网格完全一致；复制子地图未改变。',
        'Q3 首次倒车退出被 TrafficStationQ3CurbRamp 的轮胎支撑标记中止，接触法线向上（NED z≈−0.998），车身扫掠为空，已补齐已知坡道列表并复测通过；失败记录保留于 runs/ue_access_connections/batch_3_before_ground_classification/。',
        '通过批次的车身障碍接触和车辆重叠均为 0；轻量检查与实际车身静态扫掠均通过。最小曲率半径约 5.9 m，倒车仅用于泊位接入，仍需预约占用区并停稳换向。全园区自动调度保持关闭；不把局部接入通过等同于所有节点对的完整运输验收。','',
        '复现时使用系统 Python，按组串行启动夹具；组内多车并行：','', '```bash',
        '/usr/bin/python3 scripts/validate_ue_shared_transport.py --scenario config/runtime/industrialarea_ue_access_batch_1.json --output runs/ue_access_connections/batch_1',
        '/usr/bin/python3 scripts/validate_ue_shared_transport.py --scenario config/runtime/industrialarea_ue_access_batch_2.json --output runs/ue_access_connections/batch_2',
        '/usr/bin/python3 scripts/validate_ue_shared_transport.py --scenario config/runtime/industrialarea_ue_access_batch_3.json --output runs/ue_access_connections/batch_3',
        '.venv/bin/python scripts/summarize_ue_access_connections.py','```','']
    (ROOT/'docs/ue_access_connections_acceptance.md').write_text('\n'.join(lines))
    print('ACCESS CONNECTIONS PASS',len(connections),'ports',len(reverse_sites),'reverse accesses')


if __name__=='__main__':main()
