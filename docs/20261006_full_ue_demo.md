# 2026-10-06 完整 UE 演示

归档更新：旧测试记录已移至 [历史测试目录](/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421)；最新完整测试仍为 `runs/ue_full_verified_20261007/`。下文的历史复核命令记录当时的输入、输出路径。

工程：`/home/aipc/Traffic_ue4`。正常时间倍率 1，16 辆任务车辆、32 个运输任务，要求完成后全部进入等待位停车。

第一轮真实 UE 运行在 984.491 秒、17/32 个任务时失败：Car_0_0 与 Car_0_1 形成局部等待环。车辆碰撞和车身障碍接触均为 0，最大路径偏差 0.855 米，车辆、API 和保存的地图均已恢复。证据在 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据备份/Global_Traffic_Management_历史实验_20261006_232511/runs/right_turn_j_0_0_fix_20261006_194524/full/physical_acceptance.json`。该轮不算完整通过。

修正 `src/logistics_park/runtime/task_execution.py`：目的泊位预约释放后，缓存的等待位置不得越过当前泊位准入线；主动准入模式下，车辆进入准入线之后实际启动已获许可的连续动作时，保留完成该动作所需的泊位预约。保留原有碰撞、路径偏差和局部停滞标准。

新增回归 `tests/test_released_berth_hold.py` 和实际路线夹具 `tests/fixtures/ue_20261006_released_berth_hold.json`；全套 523 项测试通过。

重新预检和完整演示输出：`runs/ue_full_demo_20261006_2210/`。运行阶段见 `supervision.json`，真实 UE 进度见 `physical_progress.json`，最终结果见 `physical_acceptance.json`。最终真实 UE 结果为 PASS：32/32 个任务全部完成，16 辆车全部停车，耗时 2029.551 秒（约 33 分 50 秒）。车辆碰撞、车身障碍接触、未授权通行及道路容量违规均为 0；任务执行中没有瞬移。全程最大轨迹偏差 1.054 米，原分段路径容差保持不变。车辆、API、地图和编辑器均恢复，恢复后最大车速为 0。轻量预检同样 PASS，32/32 任务并全部停车，1359.5 模拟秒。

完整 UE 视口录屏：`runs/ue_full_demo_20261006_2210/ue_full_demo.mp4`。实际轨迹：`physical_trajectory.jsonl`；调度和控制诊断：`physical_control_diagnostics.jsonl`；汇总：`acceptance_summary.json`。第一轮失败证据保留，未覆盖。

历史实验数据已归档，最近一次完整演示的数据现位于工程同级目录 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/demo/`。后续停车姿态复核及修正见 [20261007_behavior_audit.md](20261007_behavior_audit.md)。原中断预检已移至归档目录；本轮 UE 场景和车辆状态快照见实验目录内 `runtime_snapshot/`。
