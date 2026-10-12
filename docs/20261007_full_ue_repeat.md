# 2026-10-07 完整 UE 复跑

归档更新：旧测试记录已移至 [历史测试目录](/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421)；最新完整测试仍为 `runs/ue_full_verified_20261007/`。下文的历史复核命令记录当时的输入、输出路径。

实际 UE 工程为 `/home/aipc/Traffic_ue4`。本次使用修复后的停车控制，16 辆任务车、32 个运输任务，正常时间倍率 1，要求所有任务完成后全部进入等待位停车。控制器继续执行真实车身扫描、原生碰撞分类、通行许可、道路容量和局部停滞检查，任务中不使用瞬移。

## 第一轮结果与改道修复

第一轮在 955.191 秒、17/32 个任务时失败。车辆碰撞、车身障碍接触和道路容量违规为 0，局部未授权进入计数为 1：`Car_3_0 actual local entry without a live passage 28`。该轮不算通过。证据位于 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/ue_behavior_fix_20261007/physical_acceptance.json`，录屏为同目录的 `ue_full_demo.mp4`。启动时缺少清理后未生成的临时目录，已在运行入口补齐，首次启动记录见 `startup_attempt_1/`。

实际原因是停车改道改变了历史路口与未来入口的动作分组。Car_3_0 从 Q37 前往 W5-unloading，原路口动作位于路线 station 113.252–128.381 米；车辆已驶过清尾位置 134.381 米，停在约 134.430 米。新的 Q30 入口与已通过路口间距较近，使分组变成 113.252–168.467 米，于是新计划重新要求先前路口的许可。原有点位前缀和 6 米车身前方几何检查未能发现这一许可合同变化。

`TransportController._reroute_road` 现在拒绝将已走过的动作重新扩展到当前位置的改道候选，拒绝发生在修改路线或释放容量预约之前。调度会继续尝试其他候选，原通行检查保持不变。实际路线夹具和回归见 `tests/fixtures/ue_20261007_retroactive_maneuver.json`、`tests/test_retroactive_maneuver.py`。

首次修复后全套 537 项测试已通过，其中回归覆盖保留几何前缀但改变历史许可分组的真实案例，以及拒绝时路线和容量预约不被修改。

## 新一轮

配置：`config/runtime/industrialarea_ue_full_repeat_20261007.json`。

第二轮完整轻量预检通过，真实 UE 运行未通过（见下文）。输出：`/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/ue_full_repeat_20261007/`。`supervision.json` 记录阶段；`lightweight_acceptance.json` 记录当前代码的完整预检；`physical_progress.json` 记录真实 UE 进度；`physical_acceptance.json` 为最终真实结果。录屏为 `ue_full_demo.mp4`，实际轨迹和控制诊断为 `physical_trajectory.jsonl`、`physical_control_diagnostics.jsonl`。

## 第二轮停滞与装货位许可修复

第二轮 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/ue_full_repeat_20261007/` 在 2002.828 秒、31/32 任务时触发车队有效进度停滞检查，未通过；碰撞、车身障碍接触、通行违规和容量违规均为 0。最后任务 transport-26 的 Car_1_1 位于 Q4，等待 W3-loading。Car_1_0 已于 1987.805 秒停入 Q33，但最后任务持续收到 `DEPARTURE_ADMISSION_REJECTED`（no_safe_hold）。

Q4→W3-loading 全长 92.727 米，旧准入线 74.927 米位于受保护的装货位离开区域；路线也没有可达的更早安全停靠点。旧逻辑拒绝出发，同时要求车辆先接近这个不可达的准入线才取得装货位许可。`_berth_gate` 现在对没有安全中途停靠点的路线返回源头准入线 0：车辆继续留在已有源点，待调度确认目标位无阻挡并授予许可，再出发到真正目标。目标占用、路线许可、车身碰撞及停滞阈值保持不变。

`tests/test_unreachable_admission_gate.py` 使用实际 Q4→W3 路线验证无安全中途停靠、未获目标许可禁止出发、获许可后选择实际目标，以及目标有其他车辆时仍禁止出发。全套 538 项测试通过。

第三轮配置 `config/runtime/industrialarea_ue_full_final_20261007.json`，输出 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/ue_full_final_20261007/`；完整预检通过，真实 UE 未通过（见下文）。

## 第三轮队列循环与实际前后顺序修复

第三轮 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/ue_full_final_20261007/` 轻量预检通过，但真实 UE 在 966.984 秒、17/32 时触发 Car_0_1 与 Car_7_0 的局部循环等待，未通过。Car_7_0 位于 (-12.103,161.761)，Car_0_1 位于 (-12.094,152.175)，均朝向北、同车道且前后相隔约 9.586 米；两车同去 W6-loading，原位车辆 Car_0_0 正在离开。队首意图却选了后车 Car_0_1，导致前车依赖后车队首许可，后车依赖前车物理让路。

`TaskDispatcher.promote_approaches` 现将已有的同目的地实际前车关系同时用于队首意图筛选，而不只是后续目标位授予。保留已有物理持有者的许可，优先选择没有另一候选前车的车辆。回归覆盖装货位仍被离开车辆占用、腾空后的授予，以及后车 ETA 优势不得倒置物理队列。全套 539 项测试通过。

第四轮配置 `config/runtime/industrialarea_ue_full_verified_20261007.json`，输出 `runs/ue_full_verified_20261007/`，完整验证已通过。

## 最终通过结果

第四轮真实 UE 验收为 **PASS**：32/32 任务完成、16/16 车辆在等待位停车，正常时间倍率 1，耗时 1725.769 秒（约 28 分 46 秒）。任务执行中瞬移为 0，车辆碰撞、车体障碍接触、未授权进入和道路容量违规均为 0。全程未触发停滞检查，最终等待依赖无循环。车辆、API、地图与编辑器状态均恢复，恢复后最大速度为 0。

轨迹复核：16 辆车最终停车最大朝向偏差 2.147461°、最大位置偏差 0.263837 米、最大速度 0.000235 m/s，全部满足既定 5°、0.35 米、0.08 m/s 阈值。全部服务到达的最大朝向偏差约 2.155853°。直线路段整车范围检查共 67,838 个样本，越界标记为 0、最小边界余量约 0.203810 米。低频范围检查另有 6 个接入口车尾合流标记；其范围不属于整车已进入直线路段的样本。全程最大路径投影误差 0.740304 米，未触发原有分路段限值。实际连续起步响应最长约 1.104339 秒。原生记录中的 2505 个地面支撑接触样本单独分类，不是车体障碍接触。

上述范围审计使用录制的拓扑和车辆姿态，直线检查不等同于每帧车轮射线检测；转弯与合流期间仍执行原有车身扫描、原生碰撞分类和路线误差检查。

启动前编辑器重开，Python 远程执行被关闭且打开了另一地图副本；已恢复远程连接并加载本实验原地图。地图文件指纹变化触发验收拒绝后，对旧、新场景的 2220 个对象逐项比较，无对象增删，位置、旋转、类、模型多重集合和道路标记均一致，只有枚举顺序和动态线缆包围盒变化。证据 `map_scene_comparison.json`。仅更新本轮配置的 `map_sha256`，仿真输入和算法未变；轻量报告保留了配置元数据重新绑定说明、前后指纹和旧报告。启动连接、地图基准尝试分别已归档至 `/mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/实验数据/Global_Traffic_Management_历史测试_20261007_100421/Global_Traffic_Management/runs/ue_full_verified_20261007/startup_connection_attempt/`、同级 `startup_map_baseline_attempt/`。物理入口的地图一致性校验与结束时地图未变校验均保留并通过。

最新输出：`runs/ue_full_verified_20261007/`。完整录屏 `ue_full_demo.mp4` 长 1731.75 秒（包含启动和收尾）；16 倍速 `ue_full_demo_16x.mp4` 长 108.25 秒，均为 H.264。汇总见 `acceptance_summary.json`，原始物理报告 `physical_acceptance.json`，姿态复核 `behavior_audit.json`，高频道路范围复核 `road_envelope_audit.json`。运行时场景与车辆状态见 `runtime_snapshot/`。
