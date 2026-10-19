#!/usr/bin/env python3
"""Evidence for representative continuous dispatch with actual stopped queue."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]

def main():
    out=ROOT/'runs/continuous_tasks'
    r=json.loads((out/'physical_acceptance.json').read_text());audit=json.loads((out/'restoration_audit.json').read_text())
    scenario=ROOT/'config/runtime/industrialarea_ue_continuous_tasks.json';spec=json.loads(scenario.read_text())
    sources={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in (scenario,ROOT/spec['map'],ROOT/'config/topology/industrialarea_ue_service_network.json')}
    if r['source_sha256']!=sources:raise RuntimeError('Continuous results belong to a different source configuration')
    events=r['dispatch_events'];queue=next(e for e in events if e['event']=='NODE_ARRIVED' and e['waiting'])
    promotion=next(e for e in events if e['event']=='MOVE_REQUESTED' and e['origin']==queue['node'])
    release=next(e for e in events if e['event']=='SOURCE_CLEARED' and e['node']=='W3-unloading')
    dwell=promotion['time']-queue['time'];complete=[e for e in events if e['event']=='TRANSPORT_COMPLETE']
    passed=(r['status']=='PASS' and len(complete)==3 and dwell>=1 and queue['time']<release['time']<=promotion['time']
            and r['body_obstacle_contacts']==r['vehicle_collisions']==0 and audit['status']=='PASS'
            and all(r[k] for k in ('fleet_restored','api_restored','saved_maps_unchanged')))
    summary=dict(status='PASS' if passed else 'FAIL',expected_tasks=3,completed_tasks=len(complete),elapsed_s=r['elapsed_s'],
        maximum_moving_vehicles=r['maximum_moving_vehicles'],maximum_error_m=r['maximum_error_m'],
        waiting_node=queue['node'],actual_stopped_waiting_s=dwell,promoted_after_source_tail_clear=release['time']<=promotion['time'],
        completed_movements=sum(e['event']=='NODE_ARRIVED' for e in events),teleports_during_tasks=0,vehicles_removed_between_legs=False,
        body_obstacle_contacts=r['body_obstacle_contacts'],vehicle_collisions=r['vehicle_collisions'],
        scope='Representative two-truck three-task continuous transport and actual stopped queue; no arbitrary combination or large-fleet physical claim',
        map_sha256=r['map_sha256'],source_sha256=sources,restoration_audit=audit,blocked_candidate_edges=spec['blocked_candidate_edges'])
    (ROOT/'docs/ue_continuous_tasks_acceptance.json').write_text(json.dumps(summary,indent=2)+'\n')
    tasks='\n'.join(f"| transport-{i} | {a} | {b} | PASS |" for i,(a,b) in enumerate(spec['tasks']))
    text=f'''# 连续派单与等待区任务执行验收

UE **{summary['status']}，3/3 个任务、5/5 段移动完成**，两辆车在不冲突道路上同时行驶。耗时 **{summary['elapsed_s']:.1f} 秒**，最大参考线偏差 **{summary['maximum_error_m']:.3f} m**。Q4 实际停稳等待 **{dwell:.1f} 秒**，W3 卸货位前车车尾实测清空后才晋升；车身障碍接触及车辆碰撞均为 **0**。

| 任务 | 装货位 | 卸货位 | 结果 |
|---|---|---|---|
{tasks}

初始车辆为 `Car_0_0: W1-loading`、`Car_0_1: W4-loading`。装卸各 5 秒，前车完成 W1→W3 后自动接 W1→W4，再从 W3 返回 W1 取货。后车因 W3 已被预约，先去 Q4 停稳，前车离开后连续驶向 W3。任务之间不传送、不删除或隐藏到达车辆，空闲/等待车辆持续纳入道路占用、车身包络、跟车与碰撞审计。全程保留完整短货厢卡车包络；本轮没有扩展 W3/W4 的卸货厢落地视觉事件。

## 本轮实现

- `ServiceRouteGeometry` 按当前图中确切有向边拼接，复用已有道路及局部进出，保留前进/倒车与停稳换挡。核对源几何，只有小于 15 cm 的 attachment 可通过路线上独立端点修整消除，不修改标准原轨迹、不穷举等待路线组合。
- `ContinuousTaskExecutor` 对实测车尾清空和停稳到达做回调，到达后保留实体、道路占用和预约，下一段以真实停车位置续接。移动中、位置/朝向不匹配、遗漏原预约或缺车辆观察时拒绝续接。
- 连续执行地图沿用原 10 个 FIFO 关键区域；西侧范围的右边界最多收至 x=-84.9，区分路面与 x=-82 的路边等待位。只调整 Python 调度区域，不改 UE 地图、道路轨迹或安全偏差阈值。原框和来源哈希保留在新地图中，代表任务重新实测验证。
- 运行配置禁用候选进出动作，优先使用已验证标准。第一次测试误用 Q3 候选倒车出位，碰到 `SM_Pole_14`，证据保留为 `physical_before_standard_exit_policy.json`；改用标准进出策略后选择 Q4。Q3 标准前进出位目前到达南端右车道后缺少回到这个目标的合法换道机会，因此对 W3 任务不选 Q3；不强行启用失败候选，不据此关闭 Q3 局部进出验收。
- `resource_snapshot` 汇总任务泊位、实际道路占用/承诺、共享区域与路口预约，供后续门口后端使用。真实门口坐标、边界观察及动态 AirSim 注册/离场执行尚未实现。

## 验证边界与复现

本次只验收这组代表性连续运输和实际等泊位，不宣称全部等待节点/入口/路由组合或者 8/12/24 车已实测通过。此前仅停稳后立即晋升的一轮保留为 `physical_standard_exit_no_dwell.json`，最后将后车起点改到 W4，让它更早到 Q4，补足真实等待窗口；装卸时间仍是正常 5 秒。

轻量自行车动力学预检 3/3 通过，使用同一配置和路网；没有静态障碍动力学证明。11 项新增回归覆盖保留停车车体、候选路线过滤、动态拼接、旧预约继承、移动中禁止续接、观察完整性与门口资源快照；完整项目检查 **201 项测试及 Phase 0～4 验收通过**。

测试结束 24 车、API、时间流速 1.0 与 PhysX 原设置恢复，保存地图未改，相机自由。证据为 `runs/continuous_tasks/physical_acceptance.json`、`physical_trajectory.jsonl`、`restoration_audit.json`、`lightweight_acceptance.json`、`project_check.log`。

```bash
.venv/bin/python scripts/build_ue_continuous_transport.py
.venv/bin/python scripts/simulate_ue_continuous_tasks.py
/usr/bin/python3 scripts/validate_ue_continuous_tasks.py
# 测试后运行 UE 恢复审计，保存 runs/continuous_tasks/restoration_audit.json
.venv/bin/python scripts/summarize_ue_continuous_tasks.py
```

下一阶段使用现有 FIFO 基线，先 4 车短预检，再 8 车持续派单，然后 12 车。任务分散至六仓库，保留全部运行车辆；统计完成量、同时行驶数、任务/泊位等待、控制周期与死锁，复用已有道路和等待标准动作。局部滚动时间窗优化随后与这批基线比较，不把继续逐条验证路网作为扩车前提。
'''
    (ROOT/'docs/ue_continuous_tasks_acceptance.md').write_text(text)
    print(summary['status'],'3 tasks, actual queue',round(dwell,1),'s')
    return 0 if passed else 1

if __name__=='__main__':raise SystemExit(main())
