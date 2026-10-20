# Demo 运行与控制流程

Demo 的标准入口是 `run.sh`，默认 UE 工程为 `/home/aipc/Traffic_ue4/Traffic_ue4.uproject`。配置与命令见 [README](../README.md)。

## 入口与模块

```text
run.sh → scripts/run_acceptance.py
  → 建立仓库外工作副本、绑定工程依赖、保存编辑器状态
  → 加载关卡、启动 PIE、等待 AirSim 车辆停稳
  → scripts/validate_ue_continuous_tasks.py
  → 运输任务、等待位停车、独立位姿和道路包络审计
  → 结束 PIE、恢复编辑器状态、写入 acceptance_summary.json
```

调用者负责打开 UE 编辑器。run_acceptance 管理 PIE 生命周期，在异常处理和收尾阶段执行结束 PIE、编辑器状态恢复；恢复异常写入 editor_restore_error。独立冷启动实验由外部编排流程负责工程基线恢复、编辑器创建和关闭。

ContinuousTaskExecutor 协调任务执行，TaskDispatcher 管理派单和服务资源，ServiceRouteGeometry 拼接服务路网动作。TransportController 根据实测状态推进路线和检查点并计算控制命令；LocalTraffic 处理通行许可与末端预测调节；AirSim 后端执行油门、转向和刹车。

标准场景包含 24 辆车，其中 16 辆车执行 32 个运输任务。停止、等待和已完成车辆持续保留实际车身占用；资源释放以物理清空事件为依据。

## 状态转换与运动控制

改道、授权、撤销、让行、清空义务和任务段交接通过 TrafficTransition 表达，由 TransitionValidator 校验后经 LocalTraffic.commit_transition 发布。检查覆盖停车与制动边界、路线前缀、实测车身和预测扫掠、许可阶段、等待依赖及清空义务。校验通过才发布；提交前复核状态戳，发布异常回滚。

控制链按顺序计算路线速度、动作净空、通行时序、目的地等待、共享区域和路口权限、跟车、静止车身、检查点及末端轨迹调节。各阶段给出速度上限，最终指令同时满足逻辑许可和物理安全约束。具体校验见 [状态转换规则](STATE_TRANSITIONS.md)。

校验采用当前观测帧与有限预测范围。车辆后续运动会更新物理阻塞关系，持续解阻需要随控制帧重新评估。十轮实测中的一次等待环停滞，为这一方向提供了具体复现数据。

## 等待环恢复

runtime/wait_cycle_recovery.py 定期观察等待环，在车辆停止且环稳定后尝试撤销可安全停车的通行权。已进入动作、或制动边界已越过停车线的车辆保留必要权限，撤销请求返回 passage_stop_line_committed。

十轮实验第 2 轮中，Car_6_1 与 Car_7_1 相互触发 predicted_body 保护。约 918.7 秒恢复尝试得到空撤销集合，Car_6_1 的权限因动作阶段受到保护；约 928.6 秒触发局部停滞判定。持续解阻的完善重点是撤销受限时的路线与多车协调策略，根因分析继续追踪等待环首次形成的控制帧。

## 诊断

acceptance_summary.json 汇总结果，execution.log 记录初始化和执行错误。physical_control_diagnostics.jsonl 提供各阶段目标速度、local_denials、passage_denials、control_denials、许可、占用和执行进度。physical_acceptance.json 保留转换与恢复事件，physical_trajectory.jsonl 保存轨迹，native_road_history.jsonl 保存原生道路历史。

排查顺序为：确认入口与工程 → 确认任务阶段 → 找到首次降速阶段 → 核对阻塞方和许可阶段 → 追踪恢复事件。记录场景重放用于核对指定状态下的转换结果；真实 UE 多轮实验用于评估整轮执行表现。

## 实测与冻结

回归测试 942 项通过，状态转换重放 4 个场景通过。十轮冷启动实测为 9 PASS、1 FAIL，受测工程位于 `/home/aipc/Traffic_ue4_agent`。日常工程配置见 [环境说明](UE_ENVIRONMENT.md)，逐轮数据见 [验证记录](VALIDATION_20261020.md)。

冻结摘要覆盖 590 个配置、脚本、实现和测试文件。文档由 Git 提交记录，冻结清单自身采用 Git 跟踪。详见 [冻结记录](FREEZE_20261020.md)。
