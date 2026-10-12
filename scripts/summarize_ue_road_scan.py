#!/usr/bin/env python3
"""Publish component coverage independently from 132 full-route acceptance."""
import hashlib
import json
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.route_coverage import coverage_result


def main():
    path=ROOT/'config/runtime/industrialarea_ue_road_scan.json'
    plan=json.loads(path.read_text())
    source=ROOT/plan['source_scenario']
    scenario=json.loads(source.read_text())
    if hashlib.sha256(source.read_bytes()).hexdigest()!=plan['scenario_sha256'] or hashlib.sha256((ROOT/scenario['map']).read_bytes()).hexdigest()!=plan['topology_sha256']:
        raise RuntimeError('Stale scan plan; rebuild before publishing')
    folder=ROOT/'runs/ue_node_pairs'
    candidates=[p for p in (folder/'scan_acceptance.json',folder/'scan_progress.json') if p.exists()]
    report=dict(status='NOT_RUN',results=[],ue_physics_validated=False)
    evidence=None
    if candidates:
        evidence=max(candidates,key=lambda p:p.stat().st_mtime_ns)
        report=json.loads(evidence.read_text())
        if (report.get('coverage_manifest_sha256')!=hashlib.sha256(path.read_bytes()).hexdigest()
                or report.get('topology_sha256')!=plan['topology_sha256']
                or report.get('scenario_sha256')!=plan['scenario_sha256']):
            raise RuntimeError('Evidence belongs to another scan plan')
    coverage=coverage_result(plan,report['results'])
    validated=(report['status']=='PASS' and coverage['coverage_status']=='COMPLETE'
               and report.get('ue_physics_validated') and report.get('fleet_restored')
               and report.get('api_restored') and report.get('saved_maps_unchanged')
               and report.get('body_obstacle_contacts')==0 and report.get('vehicle_collisions')==0)
    publication=dict(status='PASS' if validated else report['status'],
                     scan_physically_validated=bool(validated),**coverage,
                     source_route_count=plan['source_route_count'],selected_route_count=plan['selected_route_count'],
                     selected_pair_ids=plan['selected_pair_ids'],requirement_counts=plan['requirement_counts'],
                     source_distance_m=plan['source_distance_m'],scan_distance_m=plan['scan_distance_m'],
                     topology_sha256=plan['topology_sha256'],scenario_sha256=plan['scenario_sha256'],
                     map_sha256=report.get('map_sha256'),results=report['results'],
                     elapsed_s=report.get('elapsed_s'),maximum_moving_vehicles=report.get('maximum_moving_vehicles'),
                     body_obstacle_contacts=report.get('body_obstacle_contacts'),vehicle_collisions=report.get('vehicle_collisions'),
                     fleet_restored=report.get('fleet_restored'),api_restored=report.get('api_restored'),
                     saved_maps_unchanged=report.get('saved_maps_unchanged'),restoration_audit=report.get('restoration_audit'),
                     evidence_file=str(evidence.relative_to(ROOT)) if evidence else None)
    (ROOT/'docs/ue_road_scan_acceptance.json').write_text(json.dumps(publication,indent=2)+'\n')
    light_path=folder/'scan_lightweight_acceptance.json'
    light=json.loads(light_path.read_text()) if light_path.exists() else None
    lines=['# 装卸节点共用道路的去重并行扫图','',
           f"UE 扫图状态：**{publication['status']}**；物理覆盖 **{coverage['covered_requirement_count']}/{coverage['total_requirement_count']}** 项。",'',
           f"132 条原路线合计 {plan['source_distance_m']/1000:.2f} km，去重后选择 **{plan['selected_route_count']} 条代表任务**、{plan['scan_distance_m']/1000:.2f} km，行驶距离减少 **{100*plan['distance_reduction_fraction']:.1f}%**。这不是实测耗时缩短比例。",'',
           '按完全一致的轨迹、行驶方向、速度上限、延迟转向和共享区域语义合并重复路段；覆盖相邻两段、连续三段、所有使用到的装卸入口/出口、起步、停稳到达和停稳换挡。等待区不作为起终点。范围是原 132 条路线用到的路网，不包含这些路线未用到的其他道路或等待位进出。','',
           '| 覆盖项 | 数量 |','| --- | ---: |']
    labels={'EDGE':'不同轨迹段','JOIN':'相邻两段衔接','CHAIN3':'连续三段组合','DEPARTURE':'起步入口','ARRIVAL':'停稳到达入口','GEAR_CHANGE':'换挡组合'}
    lines.extend(f'| {labels[k]} | {v} |' for k,v in plan['requirement_counts'].items())
    if light and light.get('topology_sha256')==plan['topology_sha256']:
        lines+=['',f"轻量预检：**{light['status']} {light['completed_pairs']}/{light['expected_pairs']}**，最多 {light['maximum_moving_vehicles']} 辆同时行驶，耗时 {light['wall_time_s']:.1f} s；不能作为 UE 物理证据。"]
    if validated:
        error=max(r['maximum_error_m'] for r in report['results'])
        peak=max(r['peak_speed_m_s'] for r in report['results'])
        lines+=['',f"本轮 UE 实车：**{len(report['results'])}/{plan['selected_route_count']} 通过**，耗时 **{report['elapsed_s']:.1f} s**（约 {report['elapsed_s']/60:.1f} 分钟），最多 **{report['maximum_moving_vehicles']} 辆**同时行驶。最大参考线偏差 {error:.3f} m、峰值车速 {peak:.2f} m/s；车身障碍接触 **0**、车辆碰撞 **0**。24 车及 API 恢复、保存地图未改，相机自由。"]
        if report.get('restoration_audit',{}).get('status')=='PASS':
            lines+=['','退出后只读审计确认时间流速恢复为 1.0、PhysX 原参数恢复、临时车队夹具清空；证据 `runs/ue_node_pairs/restoration_audit.json`。']
    lines+=['','运行命令：','','```bash',
            '.venv/bin/python scripts/build_ue_road_scan.py',
            '.venv/bin/python scripts/simulate_ue_node_pairs.py --road-scan',
            '/usr/bin/python3 scripts/validate_ue_node_pairs.py --road-scan --time-scale 4',
            '.venv/bin/python scripts/summarize_ue_road_scan.py','```','',
            'UE PIE 必须已运行且车队静止。执行器使用 12 辆车池，按共享冲突区域与实际车身包络滚动放行；实际并发数取决于区域是否冲突。行驶阶段不传送、不控制相机，结束恢复 24 辆车与 API。','',
            '失败任务只记问题并清空相关测试车，继续扫完其他任务，最后集中修复与补测。`scan_*` 结果与 132 条完整路线结果分开，`--road-scan --resume` 可恢复当前地图与路线一致的通过记录。','',
            '**物理覆盖完成不等于 132 条完整路线都曾连续实测通过。** 路段可以支撑后续路径组合，但不能直接推断长任务、排队、会话状态及并发调度全部可靠；这些行为另做少量针对性任务回归。这里不改宽偏差阈值、不更改地图或参考轨迹，也不向原 `config/planning/ue_node_pair_acceptance.json` 发布全通过结论。','',
            '| 代表任务 | 独占区域 |','| --- | --- |']
    lines.extend(f"| {k} | {', '.join(plan['regions_by_pair'][k])} |" for k in plan['selected_pair_ids'])
    (ROOT/'docs/ue_road_scan_acceptance.md').write_text('\n'.join(lines)+'\n')
    print(publication['status'],'scan',plan['selected_route_count'],'jobs',coverage['covered_requirement_count'],'/',coverage['total_requirement_count'])


if __name__=='__main__':main()
