# 2026-10-07 行驶与停车复核

归档更新：旧测试记录已移至 [历史测试目录](/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421)；最新完整测试仍为 `runs/ue_full_verified_20261007/`。下文的历史复核命令记录当时的输入、输出路径。

复核对象为工程同级 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/demo/` 中最近一轮真实 UE 完整演示：16 辆车、32 个任务、2029.551 秒。保留原始演示和日志，不覆盖旧验收结论。

逐条读取 `physical_trajectory.jsonl`、`physical_control_diagnostics.jsonl`、`physical_adapter.jsonl` 和到达事件，对照当前服务节点的位置与车头方向。

| 项目 | 原轮证据与判断 |
| --- | --- |
| 停车姿态 | 9 辆车最终方向偏差大于 5°；最大为 Car_2_0 在 Q30 的 14.71°。位置误差最大约 0.229 米，速度已降为 0，但仅凭位置到达不能认为停正。 |
| 车道边界 | 低频诊断的 10931 个直道投影样本中有 8 个车身超出单车道边缘的记录，全部位于装卸通道汇入道路的过渡段，车尾仍在合法通道中。进一步检查高频轨迹中整车处于直道路段内的 85424 个样本，车身包络均未越过车道边界，最小余量约 0.162 米。这项检查依据拓扑和车身包络，不能替代每帧轮胎射线测量。 |
| 跟车响应 | 连续非零起步指令（目标速度大于 0.3 米/秒）下，车辆保持静止的最长时间约 1.122 秒。低频诊断中看似多秒的迟滞，实际包含间歇停车指令，不能当作持续起步失败。 |
| 碰撞 | 原生 UE 车身扫描及车辆包络检查记录的车辆碰撞、车身障碍接触均为 0。2623 个地面接触样本已按地面支撑规则分类，不能直接当作车辆碰撞。 |
| 卡住 | 完成全部 32 个任务并停车；局部等待环与停滞检查未触发。 |

## 修正

正式服务路线记录服务节点指定的车头方向。末段 6 米在最后一次换挡之后使用车头方向和横向误差反馈，倒车时按行驶方向转换反馈符号，最后 3 米加强车头方向反馈；保留已有转向幅度和路线误差限制。到位与收车同时要求实际方向误差不超过 5°，避免斜停被判定为完成。调度准入、跟车安全距离、碰撞检查和停滞阈值均维持原标准。Q5 的真实 UE 回归证明，过早开始末段修正会切入墙体；因此保留原有转弯过程，最后 3 米再加强朝向反馈。被拒绝的诊断轮次已移至 `../实验数据备份/行为修复过程_20261007/`，未作为通过证据。

新增复核工具 `scripts/audit_ue_run.py` 和 `scripts/audit_ue_road_envelope.py`，以及原生 UE 泊位姿态回归 `scripts/validate_ue_parking_pose.py`。原生回归按原轮所用的 16 个等待位，各运行入口末段 12 米，单车最高目标速度 1.2 米/秒，使用正式路线转向函数和到位检查，检查真实车身扫描并恢复 UE 编辑器。该回归不等于重新运行全部 32 个真实 UE 运输任务。

## 验证

最终控制参数的全套 536 项测试已通过。原生 UE 的 16 个等待位回归全部通过，最大方向误差 3.821°，最大位置误差 0.239 米，入口末段最大路线误差 0.341 米，无车身接触，编辑器已恢复。完整轻量预检 PASS：32/32 个任务完成，16 辆车全部停车，1359.4 模拟秒，碰撞、未授权通行和道路容量违规均为 0，所有最终泊位方向误差符合 5° 标准。没有重跑全部 32 个真实 UE 运输任务，原生验证范围为上述停车专项。最终数据见 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/behavior_audit_20261007/native_parking.json`、`tests.log` 和 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/ue_behavior_fix_20261007/lightweight_acceptance.json`。复核原始结果见 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/behavior_audit_20261007/before.json`。

```bash
.venv/bin/python scripts/audit_ue_run.py ../demo --output runs/behavior_audit_20261007/before.json
.venv/bin/python scripts/audit_ue_road_envelope.py ../demo --output runs/behavior_audit_20261007/road_high_frequency.json
python3 scripts/validate_ue_parking_pose.py --audit runs/behavior_audit_20261007/before.json --output runs/behavior_audit_20261007
.venv/bin/python scripts/simulate_ue_continuous_tasks.py --scenario config/runtime/industrialarea_ue_behavior_fix_20261007.json
```

最终汇总：`/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/behavior_audit_20261007/summary.json`。保存的 UE 地图哈希和原完整演示相同，原始 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/demo/` 数据未覆盖。


后续已完成包含这些停车修复的完整真实 UE 验收：32/32 任务、16/16 车辆停车，最大最终朝向偏差 2.147°、位置偏差 0.264 米，碰撞、车体障碍接触和通行违规均为 0。最新报告位于 `runs/ue_full_verified_20261007/acceptance_summary.json`，复跑期间发现并修复的调度问题见 [完整复跑记录](20261007_full_ue_repeat.md)。
