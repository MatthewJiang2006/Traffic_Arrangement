# 当前 Demo 的运行与控制流程

本文对应 2026-10-20 冻结的 Demo。默认 UE 工程为 `/home/aipc/Traffic_ue4/Traffic_ue4.uproject`；Traffic_Arrangement 与 agent 工程是迁移前开发和测试背景，默认运行不连接 agent 工程。具体命令与输出位置见 [README](../README.md)。

## 入口与模块

```text
run.sh → scripts/run_acceptance.py
  → 仓库外工作副本、工程依赖绑定与编辑器状态记录
  → 加载关卡、启动 PIE、等待 AirSim 车辆停稳
  → scripts/validate_ue_continuous_tasks.py
  → 任务完成及停车、独立位姿和道路包络审计
  → 结束 PIE、恢复编辑器状态、acceptance_summary.json
```

run_acceptance 的异常处理会尝试结束 PIE，并在已保存编辑器状态时尝试恢复；恢复失败会记录 editor_restore_error。不是所有异常都能保证恢复成功。入口不会自动重启编辑器或恢复完整工程基线。默认不跑预检不等于取消底层验收中的原生依赖、几何及安全检查。

任务执行由 runtime/task_execution.py 中的 ContinuousTaskExecutor 协调，任务分派和资源管理位于 scheduling/task_dispatch.py。ServiceRouteGeometry 拼接服务路网动作；TransportController 根据实测状态推进路线和检查点并计算控制命令；LocalTraffic 处理通行许可与末端预测调节；AirSim 后端执行车辆控制。

每轮实验固定 32 个运输任务、16 辆参与车辆；PIE 初始夹具中有 24 辆车。停止、等待、已完成车辆仍需遵循实际占用与车体安全约束，逻辑任务完成不能当作实车已清空。

## 共同状态转换与后续运动

改道、授权、撤销、让行、清空义务和任务段交接通过 TrafficTransition 表达，由 TransitionValidator 校验后经 LocalTraffic.commit_transition 发布。主要检查停车与制动边界、保留路线前缀、实测车身和预测扫掠、许可阶段、等待依赖及清空义务。拒绝不提交；提交前复核状态戳，发布异常回滚。详见 [校验规则](STATE_TRANSITIONS.md)。

共同转换通过后，控制仍依次受速度、动作净空、通行与时序、目的地等待、共享区域/路口权限、跟车、静止车身、检查点及末端 local regulation 约束。各阶段可进一步降低速度。获得通行权不等于后续每帧一定有正速度命令；逻辑许可和物理可执行性必须同时满足。

当前校验提供观测帧及有限预测范围内的可行性检查，不维护一个覆盖后续所有车辆运动的全局可执行解阻证明。底层轨迹保护可能产生新的物理等待，现有恢复也不能保证解除所有等待环。这是已知缺口，不能把共同校验描述为“此类停滞已经根治”。

## 等待环恢复的实际能力

runtime/wait_cycle_recovery.py 定期观察等待环，待车辆停止且环稳定后尝试撤销可安全停车的通行权。进入动作或已经无法在停车线外停下的权限不可强撤，拒绝原因是 passage_stop_line_committed。该恢复路径没有保证在撤销不可行时自动找到倒车、改道或多车协同脱困方案。

十轮实验第 2 轮中，Car_6_1 与 Car_7_1 受到相互 predicted_body 拒绝；约 918.7 秒恢复尝试未撤销任何通行权，其中 Car_6_1 因已承诺/进入动作而拒绝撤销；约 928.6 秒触发局部停滞失败。它说明安全约束仍可能导致无法推进。等待环首次如何形成尚未精确定位，不能把路径断言、环境错配和此运行中停滞归为同一个原因。

## 诊断证据

acceptance_summary.json 汇总结果；初始化失败先检查 execution.log。physical_control_diagnostics.jsonl 提供各阶段目标速度、local_denials、passage_denials、control_denials、许可、道路占用和执行进度。physical_acceptance.json 中保留状态转换及等待环恢复等事件；physical_trajectory.jsonl 保存物理轨迹，native_road_history.jsonl 保存原生道路历史。

排查顺序：确认入口与 UE 工程一致；确认是否真正进入任务阶段；对运行中失败比较第一个速度被压低的阶段、阻塞方、许可阶段与随后恢复事件。只看末尾等待环不能确定最早根因。历史重放证明指定输入下的转换结果，不证明整轮真实物理执行通过。

## 当前证据

942 项测试及四个历史场景重放通过。agent 工程十轮为 9 PASS、1 FAIL；原 Demo UE 的迁移后多轮结果不能用这些轮次替代。冻结摘要覆盖 590 个 config/scripts/src/tests/run.sh 文件，文档由 Git 提交记录，摘要清单不包含自身。验证细节见 [验证记录](VALIDATION_20261020.md)，冻结范围见 [冻结记录](FREEZE_20261020.md)。
