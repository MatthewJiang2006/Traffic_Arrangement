#!/usr/bin/env python3
"""Require physical delivery, stationary service, tail release and fleet restoration."""
import json
import math
import sys
from pathlib import Path
from validate_ue_loading_access import map_fingerprints
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src'))
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.ue_transport import configured_plans
CASES={'single':'ue_interwarehouse_single','dual':'ue_interwarehouse_dual','continuous':'ue_interwarehouse_continuous'}
rows=[];maps=map_fingerprints()
for label,directory in CASES.items():
    folder=ROOT/'runs'/directory
    if not (folder/'acceptance.json').exists():
        rows.append(dict(case=label,status='PENDING'));continue
    fixture=json.loads((folder/'acceptance.json').read_text());raw=json.loads((folder/'dispatch/acceptance.json').read_text());scenario=json.loads((folder/'scenario.json').read_text())
    controller=raw['controller'];events=controller['task_events'];samples=raw['samples']
    deliveries=sum(e['event']=='UNLOADING_COMPLETE' and e['site']=='W6-unloading' for e in events)
    loading=sum(e['event']=='LOADING_COMPLETE' and e['site']=='W5-loading' for e in events)
    body=sum(bool(b['static_sweep_hit']) for s in samples for a in s['body_audits'].values() for b in a['bodies'])
    overlaps=sum(bool(b['parked_vehicle_overlap']) for s in samples for a in s['body_audits'].values() for b in a['bodies'])
    releases=[]
    for e in [e for e in events if e['event']=='SHARED_RESOURCE_RELEASED']:
        sample=min(samples,key=lambda s:abs(s['t_s']-e['time']))
        releases.append(dict(vehicle=e['vehicle_id'],time_s=e['time'],clear=abs(sample['t_s']-e['time'])<.01 and not sample['channel_body_overlap'][e['channel_id']][e['vehicle_id']]))
    service=[]
    for name in scenario['routes']:
        starts=[e for e in events if e.get('vehicle_id')==name and e['event']=='SERVICING' and not e['test_station']]
        ends=[e for e in events if e.get('vehicle_id')==name and e['event'] in ('LOADING_COMPLETE','UNLOADING_COMPLETE')]
        for start,end in zip(starts,ends):
            phase=[s for s in samples if start['time']<=s['t_s']<=end['time']]
            service.append(dict(vehicle=name,site=start['site'],duration_s=end['time']-start['time'],maximum_speed_m_s=max(abs(s['speeds_m_s'][name]) for s in phase)))
    expected=len(scenario['routes'])*scenario['cycles_per_vehicle']
    final_positions={name:samples[-1]['positions'][name] for name in scenario['routes']}
    returned=True
    # Endpoints are rear-axle coordinates; full physical truck has already cleared the yard.
    model=json.loads((ROOT/scenario['map']).read_text());edge_lookup={e['edge_id']:e for e in model['edges']}
    expected_releases=sum(len(plan.shared_visits) for plan in configured_plans(TransportMap(ROOT/scenario['map']),scenario).values())
    for name,route in scenario['routes'].items():
        returned &= math.dist(final_positions[name],edge_lookup[route['edges'][-1]]['centerline'][-1])<.25
    gear=[e for e in events if e['event']=='GEAR_CHANGE']
    reverse_frames=sum(any(v<-.08 for v in s['speeds_m_s'].values()) for s in samples)
    parallel=sum(sum(abs(v)>.1 for v in s['speeds_m_s'].values())>=2 for s in samples)
    task_releases=[e for e in events if e['event']=='TRANSPORT_TASK_RELEASED']
    cycle_returns=[e for e in events if e['event']=='TRANSPORT_CYCLE_RETURNED']
    cycle_order=True
    for name in scenario['routes']:
        released=[e for e in task_releases if e['vehicle_id']==name]
        returned_cycles=[e for e in cycle_returns if e['vehicle_id']==name]
        cycle_order &= [e['cycle'] for e in released]==list(range(scenario['cycles_per_vehicle']))
        cycle_order &= [e['cycle'] for e in returned_cycles]==list(range(scenario['cycles_per_vehicle']))
        cycle_order &= all(a['time']<b['time'] for a,b in zip(released,returned_cycles))
        cycle_order &= all(a['time']<=b['time'] for a,b in zip(returned_cycles,released[1:]))
    idle_restored=(not fixture.get('idle_visibility') or (fixture.get('idle_visibility_restore',{}).get('status')=='RESTORED' and fixture['idle_visibility_restore']['actors']==fixture['idle_visibility']['actors']))
    visual=fixture.get('cargo_visuals')
    visual_pass=True
    if scenario.get('cargo_visuals'):
        visual_events=visual['events'] if visual else []
        attached=[e for e in visual_events if e['event']=='CARGO_ATTACHED']
        dropped=[e for e in visual_events if e['event']=='CARGO_DROPPED']
        removed=[e for e in visual_events if e['event']=='GROUND_CARGO_REMOVED']
        visual_pass=(bool(visual) and visual['status']=='RUNNING'
            and len(attached)==len(dropped)==len(removed)==expected
            and not visual['ground_props'] and not any(visual['attached_visible'].values())
            and not any(e['event']=='ERROR' for e in visual_events)
            and all(5<=b['monotonic_s']-a['monotonic_s']<5.5 and a['id']==b['id']
                    and abs(a['bottom_z_m']-a['ground_z_m']-.01)<1e-6 for a,b in zip(dropped,removed))
            and fixture.get('cargo_visual_cleanup',{}).get('restored'))
    passed=(fixture['status']==raw['status']=='PASS' and fixture['map_sha256']==maps and raw['ue_physics_validated'] and all(fixture[k] for k in ('fleet_restored','api_restored','settings_unchanged','saved_maps_unchanged'))
            and deliveries==loading==expected and not body and not overlaps and returned and len(service)==2*expected
            and all(s['duration_s']>=5 and s['maximum_speed_m_s']<.08 for s in service)
            and len(releases)==expected_releases and all(r['clear'] for r in releases)
            and len(gear)==len(scenario['routes'])*(scenario['cycles_per_vehicle']*4-1) and all(abs(e['speed_m_s'])<.08 for e in gear)
            and reverse_frames>0 and (label!='dual' or parallel>0) and cycle_order and visual_pass and idle_restored
            and all(not names for names in controller['service_occupants'].values()) and not any(controller['cargo_units'].values()))
    rows.append(dict(case=label,status='PASS' if passed else 'FAIL',deliveries=deliveries,expected_deliveries=expected,cycles_per_vehicle=scenario['cycles_per_vehicle'],vehicles=len(scenario['routes']),straight_target_m_s=model['speed_limits']['straight_m_s'],runtime_s=controller['simulation_time_s'],maximum_speed_m_s=max(abs(v) for s in samples for v in s['speeds_m_s'].values()),maximum_path_error_m=controller['maximum_path_error_m'],body_obstacle_hits=body,vehicle_overlap_samples=overlaps,service=service,gear_changes=gear,tail_release_proof=releases,returned_to_waiting=returned,task_release_events=task_releases,cycle_return_events=cycle_returns,cycle_order_valid=cycle_order,reverse_frames=reverse_frames,parallel_frames=parallel,fleet_restored=fixture['fleet_restored'],cargo_visuals_validated=visual_pass if scenario.get('cargo_visuals') else None,cargo_visuals=visual,idle_visibility_restored=idle_restored,camera_changed=False,evidence=str(folder.relative_to(ROOT))+'/acceptance.json'))
summary=dict(status='PASS' if all(r['status']=='PASS' for r in rows) else 'FAIL' if any(r['status']=='FAIL' for r in rows) else 'PENDING',cases=rows,map_sha256=maps,ue_physics_validated=all(r['status']=='PASS' for r in rows),full_park_automatic_dispatch=False,scope='Two approved warehouses, single and dual one delivery each; single vehicle two successive deliveries and waiting returns')
(ROOT/'docs/ue_interwarehouse_acceptance.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
lines=['# W5 → W6 跨仓库连续运输验收','',f"状态：**{summary['status']}**。装货与卸货分别位于 W5 和 W6；货物以任务状态 0 → 1 → 0 表示，未新增货厢内货物模型。",'',
       '车辆从已审批等待位出发，W5 前进装货 5 秒，驶至 W6 路边停稳换倒挡，倒车入卸货位，卸货 5 秒；停稳换前进挡后驶离，沿现有拓扑许可的道路与末端掉头返回，在入口停稳、倒车回等待位。等待保持 3 秒。行驶阶段不瞬移、不复位。', '',
       '当前跨仓场景道路直行目标 8 m/s（28.8 km/h），按 0.6 m/s² 的减速预算提前降低路口速度；装卸接近段保持 2/3 m/s。转弯 1.5 m/s，倒车及末端掉头 1 m/s。W5/W6 紧邻的作业和倒车区域共用一个预约资源，车身及车尾全部清空后释放；出口短段的南行对向车道纳入预约，以避免尚未摆正的车头与对向车辆碰撞；其他普通道路不纳入该资源，路口采用 FCFS。', '',
       '| 场景 | 状态 | 跨仓交付 | 运行秒数 | 直行目标 m/s | 实测最高 m/s | 同时移动采样数 |','|---|---|---:|---:|---:|---:|---:|']
for r in rows:
    if r['status']=='PENDING':lines.append(f"| {r['case']} | PENDING | — | — | — | — | — |")
    else:lines.append(f"| {r['case']} | {r['status']} | {r['deliveries']} | {r['runtime_s']:.1f} | {r['straight_target_m_s']:.1f} | {r['maximum_speed_m_s']:.2f} | {r['parallel_frames']} |")
lines+=['','single 保留首次单车基础闭环记录（直行目标 4 m/s，未启用货厢视觉效果）；dual 与 continuous 验证修正后的对向出口预约、8 m/s 直行目标和货厢效果。continuous 的每轮均覆盖同一单车闭环。单车从 Q2 出发并返回 Q2；双车先放行 Q2，再放行 Q1，第一辆返回 Q1、第二辆返回 Q2，以免停车车辆挡住后车倒车通道。连续场景为同一辆车完成两轮运输，每轮实际返回 Q2 等待位、停稳后开始下一轮。车辆运行路径使用现有合法路网，单轮约 576 m；两仓库虽相邻，返回绕行较长，未新增未经验证的就地调头。','',
        '## 复现','', 'UE PIE 运行、24 辆车静止，逐条执行；同一个 UE 世界不得同时跑多个夹具。','', '```bash']
for key,suffix in [('single','1v_1cycles'),('dual','2v_1cycles'),('continuous','1v_2cycles')]:
    lines.append(f'/usr/bin/python3 scripts/validate_ue_shared_transport.py --scenario config/runtime/industrialarea_ue_interwarehouse_{suffix}.json --output "$PWD/runs/{CASES[key]}"')
lines+=['/usr/bin/python3 scripts/summarize_ue_interwarehouse.py','```','',
        '开场隐藏全部闲置车辆、移开测试路线附近待命车辆并布车，结束恢复测试前全部 24 辆车姿态、可见性和 API；地图及 AirSim 设置保持不变、镜头自由。验收要求完整完成运输和返回、装卸期间静止、换挡前停稳、车身无障碍接触、无车辆重叠、预约和泊位全部释放。轮胎轻微离地只记录。','',
        '装卸视觉效果由正式服务开始事件驱动：进入装货位并停稳后显示原短货厢；卸货服务开始时隐藏车载货厢，在 W6 附近空地 [-36,190] m 放置同样的货厢，实际地面探测及静态包络检查通过后展示 5 秒并隐藏。空车头沿回程行驶，下一轮装货时重新显示。结束恢复原外观并删除临时展示对象。此效果不改变物理底盘，碰撞验收继续使用完整货车包络。', '',
        '本轮覆盖两个仓库的固定任务及两轮预先配置的连续任务；尚未启用六仓库随机任务、无限循环任务或全园区自动调度。后续扩展任务生成、其他仓库和异常恢复。']
(ROOT/'docs/ue_interwarehouse_acceptance.md').write_text('\n'.join(lines)+'\n')
print(summary['status'],[(r['case'],r['status']) for r in rows])
