# Demo · 园区运输调度

Demo 实现园区车辆的装货、运输、卸货、排队和等待位停车，包含 Python 调度算法、服务路网、车辆控制、AirSim 接口和 UE 验收工具。改道、通行授权、撤销、让行与任务段交接共用状态转换校验。

标准实验使用 16 辆车连续完成 32 个运输任务，时间倍率为 1。运行中检查车体接触、通行许可、道路容量、停车姿态和局部进度。

## 实验数据与视频下载

实验数据、原始日志、验收报告和演示视频通过百度网盘分享：

- 分享文件：**demo**
- 下载链接：[百度网盘 · demo](https://pan.baidu.com/s/19O_M7CYBkYUeR5J2xVH-1w?pwd=2006)
- 提取码：**2006**

各次实验的时间、工程路径、配置和结果以对应报告为准。视频提供完整演示和加速播放。

## 运行

准备 UE4.27、AirSim、Python 客户端和 UE Python 远程执行。打开 `/home/aipc/Traffic_ue4/Traffic_ue4.uproject`，等待编辑器加载完成，保持单实例、编辑模式。

在 Demo 目录运行：

```bash
./run.sh
# 指定仓库外的新输出目录
./run.sh --output /absolute/external/run01
```

也可显式指定工程和完整入口路径：

```bash
TRAFFIC_UE_PROJECT_DIR=/home/aipc/Traffic_ue4 \
  /mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/Demo/run.sh
```

`run.sh` 优先使用 `.venv/bin/python`，其余环境使用 python3，并调用 `scripts/run_acceptance.py`。

| 配置 | 默认值 | 设置方式 |
|---|---|---|
| UE 工程目录 | `/home/aipc/Traffic_ue4` | TRAFFIC_UE_PROJECT_DIR |
| 结果根目录 | 仓库外 `../Demo_runs/` | DEMO_OUTPUT_ROOT |
| 单次输出 | `ue_acceptance_<时间戳>` | `--output` |
| 场景配置 | `config/runtime/industrialarea_ue_full_verified_20261007.json` | 固定验收配置 |

脚本建立外部工作副本，核对工程依赖，保存编辑器状态，加载关卡并启动 PIE。24 辆场景车辆就绪后，16 辆车执行运输任务。任务与停车完成后执行独立位姿、道路包络审计，结束 PIE，并恢复关卡、相机、自动保存和节流设置。恢复异常记录在 editor_restore_error。验收期间 AirSim 数据录制保持关闭。

轻量预检和回归测试作为独立的离线验证步骤执行。多轮冷启动实验采用“恢复工程基线 → 新开编辑器 → run.sh → 保存结果 → 关闭编辑器”的循环。

## 验证结果

- 回归测试：942 项通过。
- 状态转换重放：4 个记录场景通过。
- UE 依赖与几何资产：17 项摘要核对通过。
- 十轮冷启动实验：9 次通过、1 次等待环停滞失败。成功轮完成 32/32 个任务并停放 16 辆车；失败轮约 928.6 秒完成 18/32 个任务。

十轮实测工程为 `/home/aipc/Traffic_ue4_agent/Traffic_ue4.uproject`，日常运行工程为 `/home/aipc/Traffic_ue4/Traffic_ue4.uproject`。实验环境、逐轮结果和诊断见 [验证记录](docs/VALIDATION_20261020.md)。等待环的持续解阻是当前完善方向。

Q33 的进出连接处于关闭状态，车辆使用其余已发布泊位。Q33 开放流程包含路沿通行修复和实车验收。

## 结果复核

外部运行目录中的 acceptance_summary.json 给出最终结论。详细记录位于 `workspace/runs/ue_full_verified_20261007/`：

| 文件 | 内容 |
|---|---|
| execution.log | 执行过程和初始化错误 |
| physical_acceptance.json | 任务、停车、安全及恢复结果 |
| physical_progress.json | 运行进度 |
| physical_control_diagnostics.jsonl | 速度限制、阻塞关系和通行权限 |
| physical_trajectory.jsonl | 车辆实测轨迹 |
| pose_audit.json / road_audit.json | 独立位姿与道路包络审计 |

初始化错误优先查看 execution.log。工程路径错误核对入口绝对路径、TRAFFIC_UE_PROJECT_DIR 和编辑器实际工程；库加载错误核对进程加载文件与依赖摘要。

## 离线验证

在依赖齐备的环境中，从 Demo 目录执行：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:scripts \
  python3 -B -m pytest -q -p no:cacheprovider -p no:nose tests
PYTHONDONTWRITEBYTECODE=1 python3 -B scripts/replay_traffic_transitions.py
```

Python 字节码和测试缓存保持关闭。轻量仿真及阶段检查脚本在仓库外的可写工作副本运行，输出位于该副本的 runs。Demo 源文件采用只读保护，操作流程见 [仓库保护说明](docs/REPO_PROTECTION.md)。

## 文档与目录

- [运行与控制流程](docs/CURRENT_RUNTIME.md)
- [状态转换校验](docs/STATE_TRANSITIONS.md)
- [UE 环境与工程配置](docs/UE_ENVIRONMENT.md)
- [验证记录](docs/VALIDATION_20261020.md)
- [冻结记录](docs/FREEZE_20261020.md)
- [服务节点路网](docs/ue_service_network.md)
- [任务调度](docs/task_dispatch.md)
- [动态车队接口](docs/dynamic_fleet_interfaces.md)
- [仿真架构](docs/architecture.md)

src/logistics_park 存放调度与控制实现，config 存放路网与场景配置，scripts 提供入口和审计工具，tests 存放回归用例，docs 存放技术说明。实验日志和视频通过上述网盘链接分享。
