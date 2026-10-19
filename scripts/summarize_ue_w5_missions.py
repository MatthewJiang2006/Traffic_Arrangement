#!/usr/bin/env python3
"""Publish measured W5 local loading/unloading, berth capacity and queue clearance."""
import json
from pathlib import Path
from validate_ue_loading_access import map_fingerprints
ROOT=Path(__file__).resolve().parents[1];RAW=ROOT/'runs/ue_w5_missions'
f=json.loads((RAW/'acceptance.json').read_text());r=json.loads((RAW/'dispatch/acceptance.json').read_text());c=r['controller']
events=c['task_events'];samples=r['samples'];rows=[]
for n in ('Car_0_0','Car_0_1'):
    sequence=[e for e in events if e.get('vehicle_id')==n]
    services=[]
    for start in [e for e in sequence if e['event']=='SERVICING']:
        done=next((e for e in sequence if e['event']==start['operation']+'_COMPLETE' and e['site']==start['site']),None)
        interval=[s for s in samples if done and start['time']<=s['t_s']<done['time']]
        services.append(dict(site=start['site'],operation=start['operation'],duration_s=done['time']-start['time'] if done else 0,stationary=bool(interval) and all(abs(s['speeds_m_s'][n])<.08 for s in interval)))
    release=next((e for e in sequence if e['event']=='SHARED_RESOURCE_RELEASED'),None)
    sample=min(samples,key=lambda s:abs(s['t_s']-release['time'])) if release else None
    rows.append(dict(vehicle_id=n,initial_waiting='Q2' if n=='Car_0_0' else 'Q1',services=services,cargo_units=c['cargo_units'][n],work_area_release_verified=bool(sample and abs(sample['t_s']-release['time'])<.01 and not sample['channel_body_overlap']['W5_work_access'][n]),berths_released=sum(e['event']=='BERTH_RELEASED' and e['tail_clear'] for e in sequence)))
body_hits=sum(bool(b['static_sweep_hit']) for s in samples for a in s['body_audits'].values() for b in a['bodies'])
vehicle_hits=sum(bool(b['parked_vehicle_overlap']) for s in samples for a in s['body_audits'].values() for b in a['bodies'])
unauthorized=sum(inside and s['owners'].get(ch)!=n for s in samples for ch,v in s['channel_body_overlap'].items() for n,inside in v.items())
parallel=sum(sum(abs(v)>.1 for v in s['speeds_m_s'].values())>=2 for s in samples)
valid=(f['status']==r['status']=='PASS' and r.get('warehouse_local_missions_completed')==2 and not body_hits and not vehicle_hits and not unauthorized and parallel>0 and all(row['cargo_units']==0 and row['berths_released']==2 and row['work_area_release_verified'] and len(row['services'])==2 and all(s['duration_s']>=5 and s['stationary'] for s in row['services']) for row in rows) and f['map_sha256']==map_fingerprints())
summary=dict(status='PASS' if valid else 'FAIL',scope=r['scope'],warehouse='W5',local_missions_completed=2 if valid else 0,service_operations_completed=c['completed_tasks'],vehicles=rows,parallel_moving_samples=parallel,stationary_shared_wait_s=r['stationary_shared_wait_s'],maximum_berth_occupants=c['maximum_berth_occupants'],final_berth_occupants=c['service_occupants'],body_obstacle_hits=body_hits,vehicle_overlaps=vehicle_hits,unauthorized_work_area_entries=unauthorized,maximum_path_error_m=c['maximum_path_error_m'],maximum_speed_m_s=max(abs(v) for s in samples for v in s['speeds_m_s'].values()),runtime_s=c['simulation_time_s'],map_sha256=f['map_sha256'],fleet_restored=f['fleet_restored'],api_restored=f['api_restored'],camera_changed=False,full_park_routing_enabled=False)
(ROOT/'docs/ue_w5_missions_acceptance.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
lines=['# W5 正式双车装卸作业验收','',f"状态：**{summary['status']}**。正式 AirSim Adapter 与 `TransportController` 在已审批的 W5 点位运行两条本地作业任务：Q2 先行，Q1 等待；装货、卸货各 5 秒，驶回北行道路后停稳。这里是同仓库本地作业，不是跨仓库运输。",'', '| 车辆 | 初始等待位 | 装货服务 | 卸货服务 | 卸货后载荷 | 泊位释放 |','|---|---|---|---|---:|---:|']
for row in rows:
    text=[f"{s['duration_s']:.2f} s / {'停稳' if s['stationary'] else '未停稳'}" for s in row['services']]
    lines.append(f"| {row['vehicle_id']} | {row['initial_waiting']} | {text[0] if text else '无'} | {text[1] if len(text)>1 else '无'} | {row['cargo_units']} | {row['berths_released']}/2 |")
lines+=['',f"W5 装货/卸货位各容量 1，实测最大泊位占用 {c['maximum_berth_occupants']}，结束全部释放。装货完成后符号载荷 0→1，卸货后 1→0；货物不绘制或绑定 UE 货物模型。",'',f"独占资源只覆盖 W5 实际入口/装卸/转弯区 [-45,168,-6,184] m，Q1/Q2 等待位在该区域之外。车身全部离开并含 0.1 m 缓冲后释放通道。两车同时移动记录 {parallel} 个采样，正常道路不设共享通道独占。",'',f"车身障碍接触 {body_hits} 次、车辆重叠 {vehicle_hits} 次、未授权进区 {unauthorized} 次；最高车速 {summary['maximum_speed_m_s']:.2f} m/s，最大路径偏差 {c['maximum_path_error_m']:.3f} m，总用时 {c['simulation_time_s']:.2f} 秒。道路速度上限 4 m/s，作业直线 2 m/s，入口转弯 1.5 m/s，接近泊位自动减速。",'', '## 重现','', '当前 PIE 运行且车队静止：','','```bash','/usr/bin/python3 scripts/validate_ue_shared_transport.py \\', '  --scenario config/runtime/industrialarea_ue_w5_missions.json \\', '  --output runs/ue_w5_missions','/usr/bin/python3 scripts/summarize_ue_w5_missions.py','```','', '开场临时布车，行驶过程中不传送、不 reset；结束恢复 24 辆车、API 控制及点位标记，相机自由。直接运行正式 CLI 要求起点和朝向匹配；仓库布局、来源路线及地图哈希不一致时夹具拒绝执行。','', '这轮仅验证 W5 的两条本地作业，尚未接入 W5→W6 等跨仓运输、从道路倒车进入泊位、跨仓装卸任务及全园区连续调度。下一步扩展真正跨仓路线并接入倒车执行。所有站点全园区自动调度开关仍关闭，测试场景单独运行。','', '原始证据：`runs/ue_w5_missions/acceptance.json`、`dispatch/acceptance.json`、`dispatch/adapter.jsonl`、`scenario.json`。']
(ROOT/'docs/ue_w5_missions_acceptance.md').write_text('\n'.join(lines)+'\n')
print(summary['status'],'missions',summary['local_missions_completed'],'parallel samples',parallel)
if not valid:raise SystemExit(1)
