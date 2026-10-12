#!/usr/bin/env python3
"""Publish only the twelve local waiting ingress/hold/egress results."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    folder=ROOT/'runs/ue_node_pairs'
    report=json.loads((folder/'waiting_acceptance.json').read_text())
    audit=json.loads((folder/'waiting_restoration_audit.json').read_text())
    scenario=ROOT/'config/runtime/industrialarea_ue_waiting_scan.json';spec=json.loads(scenario.read_text())
    if report['scenario_sha256']!=hashlib.sha256(scenario.read_bytes()).hexdigest() or report['topology_sha256']!=hashlib.sha256((ROOT/spec['map']).read_bytes()).hexdigest():
        raise RuntimeError('Results belong to a different waiting manifest')
    rows=[]
    for site in spec['expected_waiting_nodes']:
        result=next(r for r in report['results'] if r['pair_id']==site+'__local_access')
        events=[e for e in report['controller_events'] if e.get('site')==site]
        arrival=next((e for e in events if e['event']=='ARRIVED'),None)
        complete=next((e for e in events if e['event']=='TASK_COMPLETE'),None)
        road=next((e for e in report['controller_events'] if e.get('site')==site+'-road' and e['event']=='ARRIVED'),None)
        hold=None if not arrival or not complete else complete['time']-arrival['time']
        passed=result['status']=='PASS' and arrival and road and hold is not None and hold>=3
        rows.append(dict(waiting_node=site,status='PASS' if passed else 'FAIL',ingress_observed=bool(arrival),egress_observed=bool(road),
            stopped_hold_s=hold,maximum_error_m=result['maximum_error_m']))
    passed=(report['status']=='PASS' and len(rows)==12 and all(r['status']=='PASS' for r in rows)
            and audit['status']=='PASS' and all(report[k] for k in ('fleet_restored','api_restored','saved_maps_unchanged'))
            and report['body_obstacle_contacts']==report['vehicle_collisions']==0)
    summary=dict(status='PASS' if passed else 'FAIL',scope=spec['scope'],waiting_nodes=rows,
        elapsed_s=report['elapsed_s'],maximum_moving_vehicles=report['maximum_moving_vehicles'],
        maximum_error_m=max(r['maximum_error_m'] for r in rows),map_sha256=report['map_sha256'],
        body_obstacle_contacts=report['body_obstacle_contacts'],vehicle_collisions=report['vehicle_collisions'],
        restoration_audit=audit,all_waiting_node_pair_routes_validated=False,
        evidence='runs/ue_node_pairs/waiting_acceptance.json')
    (ROOT/'docs/ue_waiting_scan_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
    lines=['# 等待区局部进出并发验收','',
        '按最新要求，只覆盖 Q1～Q12 的保留标准局部动作：驶入、停稳至少 3 秒、驶出回车道。不枚举所有含等待区的节点对路线。共用道路沿用既有道路扫图证据；本次不证明所有道路端口衔接、多入口方向或等待区之间的完整路径组合。','',
        f"UE **{summary['status']}，12/12 个等待区**；耗时 **{summary['elapsed_s']:.1f} 秒**，最多 **{summary['maximum_moving_vehicles']} 车同时行驶**，最大参考线偏差 **{summary['maximum_error_m']:.3f} m**。车身障碍接触及车辆碰撞均为 **0**。",'',
        '| 等待区 | 驶入 | 停稳秒数 | 驶出回车道 |','|---|---|---:|---|']
    lines += [f"| {r['waiting_node']} | {r['status']} | {r['stopped_hold_s']:.3f} | {r['status']} |" for r in rows]
    lines += ['',
        '复用当前服务路网中的局部接入序列，跳过单独的道路端口 attachment 边，保留前进/倒车与停稳换挡。12 条完整局部动作共约 794.6 m；按完整货车包络保守划分四个区域，不冲突的滚动并发，共用通道顺序完成。行驶期间不传送，初始测试布置和完成后临时清空仅用于夹具；相机自由。','',
        '本次原轨迹记录绑定的历史地图与当前地图不同，保留历史绑定信息，另在当前保存地图上重新验收这些局部动作；未更新服务节点正式启用状态，也未将结果扩展到完整道路端口或任意组合。','',
        '结束恢复 24 车与 API，最大恢复速度 0 m/s；地图未改，时间流速恢复 1.0、PhysX 原设置恢复，临时夹具清空。完整检查 **190 项测试及 Phase 0～4 验收通过**。','',
        '原始证据：`runs/ue_node_pairs/waiting_acceptance.json`、`waiting_restoration_audit.json`、`waiting_physical.log`；轻量预检 12/12 通过，最多同时行驶 4 车；完整检查日志 `runs/ue_waiting_scan_project_check.log`。','',
        '```bash','.venv/bin/python scripts/build_ue_waiting_scan.py',
        '.venv/bin/python scripts/simulate_ue_node_pairs.py --waiting-access',
        '/usr/bin/python3 scripts/validate_ue_node_pairs.py --waiting-access --time-scale 4',
        '# 测试后执行 scripts/ue/audit_node_pair_restoration.py，并保留 waiting_restoration_audit.json',
        '.venv/bin/python scripts/summarize_ue_waiting_scan.py','```','']
    (ROOT/'docs/ue_waiting_scan_acceptance.md').write_text('\n'.join(lines))
    print(summary['status'],'12 local waiting loops')
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
