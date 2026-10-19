#!/usr/bin/env python3
"""Keep failed fleet stress evidence distinct from passed local maneuvers."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    baseline=json.loads((ROOT/'runs/fleet24_tasks/physical_acceptance.json').read_text())
    audit=json.loads((ROOT/'runs/fleet24_tasks/restoration_audit.json').read_text())
    light=json.loads((ROOT/'runs/fleet24_tasks/lightweight_acceptance.json').read_text())
    rolling=json.loads((ROOT/'runs/fleet24_rolling/lightweight_acceptance.json').read_text())
    result=dict(status='FAIL',scope='24-truck sustained task stress test; prototype remains experimental',
                physical_baseline=baseline,restoration_audit=audit,lightweight_baseline=light,rolling_lightweight=rolling,
                rolling_ue_validated=False,full_rolling_time_windows_implemented=False)
    (ROOT/'docs/ue_fleet24_scheduler_status.json').write_text(json.dumps(result,indent=2)+'\n')
    cycles=rolling['wait_dependencies']['cycles']
    text=f'''# 24 车压力测试与调度优化状态

## UE 基线实测：FAIL，排队停滞

24 辆车全部保留、全部获得任务，任务池为六仓库 72 个连续运输任务。UE 最多 **{baseline['maximum_moving_vehicles']} 辆同时移动**，运行 **{baseline['elapsed_s']:.1f} 秒**后因全队超过 120 秒没有移动而停止，完成 **{baseline['completed_tasks']}/72**；不是 24 车持续运输通过。车身障碍接触 **{baseline['body_obstacle_contacts']}**、车辆碰撞 **{baseline['vehicle_collisions']}**，最大参考线偏差 **{baseline['maximum_error_m']:.3f} m**。

测试沿已验证直行道路的原有分片布置 24 个临时起点，保持装卸位空闲；任务之间不传送、不隐藏。每个起点能合法通向六个装货位，车辆之间及与既有泊位、关键区域保持包络分离。起点是测试布置，不是新增运营等待区；没有修改 UE 地图或标准轨迹。

轻量预检先发现停滞：最多 {light['maximum_moving_vehicles']} 辆移动、{light['elapsed_sim_s']:.1f} 个仿真秒、0/72 完成、车辆包络碰撞和共享区域越权均为 0。实际 UE 运行使用显式 `--diagnose-stall`：仅允许源哈希一致、失败原因确为无车辆移动且没有包络碰撞/越权的预检进入诊断；其他失败仍拒绝执行。这轮是有界诊断，不将失败预检伪装成通过。

UE 使用正常时间流速 1.0，直行目标仍是 8 m/s。控制周期均值约 {baseline['mean_control_cycle_s']:.3f} 秒、峰值约 {baseline['maximum_control_cycle_s']:.3f} 秒；不能据此承诺严格实时预算。24 辆车体监测复用同帧车身变换，仍分别保留扫掠、轮胎诊断及车辆重叠检查。相机保持自由。

结束后 24 车位置、速度和 API 恢复，保存地图未改；只读审计 **{audit['status']}**，时间流速为 1.0、PhysX 恢复、测试夹具清空，随后确认所有车辆静止。

## 已开始的调度优化

`RollingResourceReservations` 是独立、可选的滚动联合放行原型：

- 相邻关键区域在货车无法安全停在其间时联合申请，避免只先拿到半组资源。
- 根据同周期观测检查出口车身包络；出口受阻的候选暂不获准，其他可安全放行的候选可继续考虑。
- 使用预计占用时长和等待老化排序，记录承诺的预测窗口。预测结束时间不释放实际占用，仍必须实测车尾清空。
- 任务层保护已下发路线要求清空的节点和资源，等待位/闲车移位不得占住其他正在执行任务的通道。
- 等待依赖快照区分目的泊位/作业通道、路线清空、共享区域和实际前车，并识别强连通循环；诊断本身不撤销任何物理占用。

原 FIFO 配置保留，优化使用单独的配置和证据目录。原型已接入轻量/UE 执行入口，但本轮**仅做优化版轻量测试，没有运行优化版 UE**。这不是完整未来容量时间窗、候选速度搜索或 beam search 的实现。

## 优化版尚未通过

同一 24 车起点和 72 任务，原型轻量测试仍然停滞：最多 {rolling['maximum_moving_vehicles']} 车移动，{rolling['elapsed_sim_s']:.1f} 个仿真秒，完成 0/72；包络碰撞及共享区域越权均为 0。不能宣称吞吐量改善。

停止时识别的等待循环：`{cycles}`。其中 `Car_0_1` 占用 W2-loading，等待 W4-unloading；`Car_6_0` 占用 W4-loading，等待 W2-unloading。装卸位通过仓库作业入口资源互斥，造成两车互等。另一组将共享区域预约与实测前车排队连成循环。快照循环是定位证据，不直接作为撤销已进入区域许可的理由。

下一轮应先完成：

1. 装货前联合检查后续卸货容量和可用离路等待位，避免交叉占住仓库入口后再互等；保留实际泊位和车尾释放规则。
2. 将扫图时使用的较大独占区域细化成实际局部冲突区，区分正常道路、路口、泊位与共享入口，不能简单删除互斥规则。
3. 针对循环中的已停稳车辆规划标准动作疏通，保护已经进入区域的车辆；优化版通过轻量压力测试后，再做同配置 24 车 UE 对比。

## 验证与复现

完整检查 **214 项测试及 Phase 0～4 验收通过**，日志 `runs/fleet24_rolling/project_check.log`。新增验证覆盖起点合法性/分离/源几何、跟车距离筛选、联合申请、出口门控、等待老化、物理占用不随预测过期释放、已承诺路线保护和等待循环诊断。测试通过不等于 24 车任务通过。

```bash
.venv/bin/python scripts/build_ue_fleet24_tasks.py
.venv/bin/python scripts/simulate_ue_continuous_tasks.py --scenario config/runtime/industrialarea_ue_fleet24_tasks.json
# 本轮因预检只发现无碰撞停滞，执行显式限时诊断；其他失败不会放行：
/usr/bin/python3 scripts/validate_ue_continuous_tasks.py --scenario config/runtime/industrialarea_ue_fleet24_tasks.json --diagnose-stall --time-scale 1
.venv/bin/python scripts/simulate_ue_continuous_tasks.py --scenario config/runtime/industrialarea_ue_fleet24_rolling.json
.venv/bin/python scripts/summarize_ue_fleet24.py
```

基线证据：`runs/fleet24_tasks/physical_acceptance.json`、`physical_trajectory.jsonl`、`physical.log`、`restoration_audit.json`、`lightweight_acceptance.json`。第一次起点筛选未排除园外方向终点的预检保留为 `lightweight_before_reachable_starts.json`，后续已修正。优化证据：`runs/fleet24_rolling/lightweight_acceptance.json`、`light.log`；两份运行配置分别为 `config/runtime/industrialarea_ue_fleet24_tasks.json` 和 `industrialarea_ue_fleet24_rolling.json`。
'''
    (ROOT/'docs/ue_fleet24_scheduler_status.md').write_text(text)
    print('24-truck stress FAIL; first admission prototype implemented, stall unresolved; restoration',audit['status'])
if __name__=='__main__':main()
