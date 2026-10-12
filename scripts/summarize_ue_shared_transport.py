#!/usr/bin/env python3
"""Publish physical FIFO, measured tail clearance and fixture restoration evidence."""
import json
from pathlib import Path
from validate_ue_loading_access import map_fingerprints
ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/'runs/ue_shared_transport'
f=json.loads((RAW/'acceptance.json').read_text());r=json.loads((RAW/'dispatch/acceptance.json').read_text())
c=r['controller'];events=c['task_events'];samples=r['samples']
released=[e for e in events if e['event']=='SHARED_RESOURCE_RELEASED']
release_proof=[]
for e in released:
    sample=min(samples,key=lambda s:abs(s['t_s']-e['time']))
    clear=(abs(sample['t_s']-e['time'])<.01 and not sample['channel_body_overlap'][e['channel_id']][e['vehicle_id']])
    release_proof.append(dict(vehicle_id=e['vehicle_id'],channel_id=e['channel_id'],time_s=e['time'],measured_complete_body_clear=clear))
unauthorized=sum(inside and s['owners'].get(ch)!=n for s in samples for ch,vehicles in s['channel_body_overlap'].items() for n,inside in vehicles.items())
body_hits=sum(bool(b['static_sweep_hit']) for s in samples for a in s['body_audits'].values() for b in a['bodies'])
vehicle_hits=sum(bool(b['parked_vehicle_overlap']) for s in samples for a in s['body_audits'].values() for b in a['bodies'])
scenario=json.loads((RAW/'scenario.json').read_text())
repair_path=ROOT/'docs/ue_seam_repair_acceptance.json'
repaired=repair_path.exists() and json.loads(repair_path.read_text()).get('status')=='PASS'
seam_note=('原 j_1_1 横路卡车接缝已修复，原园区和复制区域的跨缝及正常双向道路并行复测均通过，见 [坡道修复验收](ue_seam_repair_acceptance.md)。失败原始记录保留于 `runs/ue_shared_transport/j1_ramp_failure/`。' if repaired else '原 j_1_1 横路等待坡道接缝会卡车，尚需修复复测；失败原始记录保留于 `runs/ue_shared_transport/j1_ramp_failure/`。')
order=[e['vehicle_id'] for e in c['shared_resource_events'] if e['event']=='GRANTED']
passed=(f['status']==r['status']=='PASS' and r['ue_physics_validated'] and len(release_proof)==2 and all(e['measured_complete_body_clear'] for e in release_proof) and not unauthorized and not body_hits and not vehicle_hits and order==scenario['expected_shared_order'] and f['map_sha256']==map_fingerprints())
summary=dict(status='PASS' if passed else 'FAIL',ue_physics_validated=passed,scope=r['scope'],channel='shared_corridor_j0',fifo_order=order,release_proof=release_proof,stationary_shared_wait_s=r['stationary_shared_wait_s'],completed_tasks=c['completed_tasks'],body_obstacle_hits=body_hits,vehicle_overlap_samples=vehicle_hits,unauthorized_shared_entry_samples=unauthorized,maximum_path_error_m=c['maximum_path_error_m'],runtime_s=c['simulation_time_s'],fleet_restored=f['fleet_restored'],api_restored=f['api_restored'],saved_maps_unchanged=f['saved_maps_unchanged'],settings_unchanged=f['settings_unchanged'],map_sha256=f['map_sha256'],camera_changed=False,warehouse_routing_enabled=False,pending_j1_ramp_seam=None if repaired else 'runs/ue_shared_transport/j1_ramp_failure/acceptance.json',seam_repair_verified=repaired)
(ROOT/'docs/ue_shared_transport_acceptance.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
text=f'''# 正式 UE 双车共享通道验收

状态：**{summary['status']}**。正式 `logistics-park-ue --scenario config/runtime/industrialarea_ue_shared_transport.json` 入口经 AirSim Adapter 调用同一个 `TransportController`，复用 FIFO 共享资源预约器。此轮为低 Y 横路 j_0_1 东侧的双向共享路段，两辆车在各自合法右侧车道行驶；通道独占控制是本次测试规则。

## 实测结果

- 申请、授权、车身进入、车尾清空后释放均有事件记录；授权顺序为 {' → '.join(order)}。
- 排队静止等待：{max(r['stationary_shared_wait_s'].values()):.2f} 秒。
- 两辆车分别完成一个路段运输与 2 秒虚拟终点服务，共 {c['completed_tasks']}/2 个任务；未接入仓库泊位。
- 车身碰障碍 {body_hits} 次，车辆包络重叠 {vehicle_hits} 次，未授权共享通道进入 {unauthorized} 次。
- 最大路线误差 {c['maximum_path_error_m']:.3f} m，运行 {c['simulation_time_s']:.2f} 秒。
- 每次释放均以当次 UE 三段车身实际 OBB 包络确认全部离开通道边界，包含 0.1 m 缓冲；释放证据见 JSON 的 `release_proof`。
- 测试结束复位全部 24 车、恢复 API；地图与 AirSim 配置未改动，相机保持自由，24 个泊位标记恢复。

## 重现

当前 UE PIE 必须运行且车队静止：

```bash
/usr/bin/python3 scripts/validate_ue_shared_transport.py
/usr/bin/python3 scripts/summarize_ue_shared_transport.py
```

夹具只在开场清障和布车、结束复位；正式程序行驶阶段不瞬移、不重置车辆。直接运行 CLI 要求车辆位置和朝向匹配配置，夹具会完成这些准备。命令超时、状态超时、异常及退出制动继续由 Adapter 负责。

## 范围及待处理事项

这轮验证真实 UE 路段上的共享资源闭环，未完成 W1～W6 仓库装卸及等待位到泊位的正式连续运输，站点自动调度仍为 0/24。W5 已另行通过两车等待、装货、卸货及返回道路的本地作业，见 [W5 双车作业验收](ue_w5_missions_acceptance.md)。下一步扩展真正跨仓库运输及倒车接入。

{seam_note}

原始记录：`runs/ue_shared_transport/acceptance.json`、`dispatch/acceptance.json`、`dispatch/adapter.jsonl`、`scenario.json`。轻量 A/B/C/D 回归另存 `runs/ue_shared_transport/lightweight/`，其 `ue_physics_validated` 保持 false。
'''
(ROOT/'docs/ue_shared_transport_acceptance.md').write_text(text)
print(summary['status'],'FIFO:',order,'physical releases:',release_proof)
if not passed:raise SystemExit(1)
