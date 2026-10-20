# Demo · 园区运输调度

本仓库包含园区物流运输的 Python 调度算法、道路与服务点拓扑、车辆控制、AirSim 接口、轻量仿真和 UE 验收脚本。当前代码已接入改道、通行授权、撤销、让行与任务段交接的共同状态转换校验。

当前冻结说明见 [2026-10-20 冻结记录](docs/FREEZE_20261020.md)，技术流程见 [当前运行说明](docs/CURRENT_RUNTIME.md)。2026-10-19 及更早文档保留对应阶段的历史结果，不代表当前代码已通过同等验证。

## 当前验证状态

- Demo 替换后：942 项回归测试通过，四个历史场景的状态转换重放通过，原 UE 的 17 项原生/几何依赖摘要核对通过。
- 被迁入版本在独立 agent UE 工程中完成 10 次同版冷启动实验：9 次通过、1 次等待环停滞失败。成功轮完成 32/32 个任务、16 辆车停放；失败轮约 928.6 秒完成 18/32 个任务。
- 上述 10 轮使用 `/home/aipc/Traffic_ue4_agent`，不能视为替换后的原 Demo UE 工程已完成多轮验收。仍有偶发停滞问题，不保证每次成功。证据与限制见 [验证记录](docs/VALIDATION_20261020.md)。

Q33 因进出路沿存在物理卡车风险，已在源拓扑关闭 Q33_entry、Q33_exit；固定 32 个任务和 16 辆参与车辆使用其余泊位。重新开放需要单独修复并进行物理验收。

## 启动真实 UE 验收

先单独打开 `/home/aipc/Traffic_ue4/Traffic_ue4.uproject`，等待编辑器加载完成。只保留一个 UE 编辑器，保持未进入 PIE 的编辑状态；脚本负责启动和结束 PIE。UE4.27、AirSim、Python 客户端及 UE Python 远程执行需预先配置。

在 Demo 目录执行：

```bash
./run.sh
# 也可指定新的仓库外输出目录
./run.sh --output /absolute/external/run01
```

避免终端所在目录或环境变量选错入口，可显式运行：

```bash
TRAFFIC_UE_PROJECT_DIR=/home/aipc/Traffic_ue4 \
  /mnt/190c1d31-5aef-e142-8902-edc8e0c6e516/Demo/run.sh
```

`run.sh` 优先使用项目 `.venv/bin/python`，否则使用 python3，调用 `scripts/run_acceptance.py`。TRAFFIC_UE_PROJECT_DIR 可覆盖工程路径，DEMO_OUTPUT_ROOT 可覆盖输出根目录；默认工程为 `/home/aipc/Traffic_ue4`，默认结果位于仓库外 `../Demo_runs/ue_acceptance_<时间戳>/`。

当前入口直接运行 1 倍速真实 UE 验收，使用 `config/runtime/industrialarea_ue_full_verified_20261007.json`，不自动运行轻量预检、回归测试或原生门禁；运行中的依赖、地图和安全检查仍保留。脚本建立外部工作副本，保存编辑器状态，加载关卡、启动 PIE 并等待 24 车就绪，其中 16 车参与运输；随后执行任务验收及独立位姿、道路包络审计，退出 PIE 并尝试恢复关卡、相机、自动保存与节流设置。任务期间不录屏，并关闭 AirSim 数据录制。

一次 run.sh 不会新开/关闭 UE 编辑器，也不恢复冷启动文件基线。需要独立多轮实验时，须在每轮前另外恢复基线、新开编辑器，结束后关闭，再进行下一轮。不能把连续在同一编辑器中运行当作独立冷启动。

不要用旧入口 `python scripts/run.py` 替代 run.sh：它先跑轻量预检，直接运行底层脚本并写入仓库内 runs，不具备当前入口的隔离与生命周期管理。

## 结果和故障检查

先查看外部运行目录中的 `acceptance_summary.json`。详细日志和轨迹在其 `workspace/runs/ue_full_verified_20261007/`：execution.log、physical_acceptance.json、physical_progress.json、physical_control_diagnostics.jsonl、physical_trajectory.jsonl，以及成功时的独立审计报告。

初始化失败可能没有 physical_acceptance.json；此时应同时看 execution.log，外层缺少报告的 FileNotFoundError 可能掩盖了里面更早的错误。工程路径断言失败时，先核对运行的是哪个目录的 run.sh、TRAFFIC_UE_PROJECT_DIR 和实际打开的 UE 工程；“原生库未加载”还需核对实际进程加载文件，不能通过关闭校验解决。

Demo 默认只读，运行数据必须留在仓库外；写保护、授权修改和恢复锁定流程见 [仓库保护说明](docs/REPO_PROTECTION.md)。

## 离线验证

已安装依赖的环境中，从 Demo 运行以下命令，不启动 UE；测试缓存和 Python 字节码均关闭：

```bash
PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:scripts \
  python3 -B -m pytest -q -p no:cacheprovider -p no:nose tests
PYTHONDONTWRITEBYTECODE=1 python3 -B scripts/replay_traffic_transitions.py
```

轻量仿真和旧阶段检查脚本默认写入 runs，仅在仓库外的可写工作副本中手动运行。不要在锁定的 Demo 内创建虚拟环境或运行会写入源码目录的旧入口。

## 技术文档

- [当前模块流程、诊断与限制](docs/CURRENT_RUNTIME.md)
- [共同状态转换校验](docs/STATE_TRANSITIONS.md)
- [2026-10-20 验证记录](docs/VALIDATION_20261020.md)
- [2026-10-20 冻结记录](docs/FREEZE_20261020.md)
- [代码替换与 UE 绑定](docs/DEMO_REPLACEMENT_20261020.md)
- [服务节点路网](docs/ue_service_network.md)
- [任务调度开发历史](docs/task_dispatch.md)
- [动态车队接口及尚未接入的范围](docs/dynamic_fleet_interfaces.md)
- [早期小车仿真架构](docs/architecture.md)

主要代码位于 src/logistics_park，配置位于 config，入口与审计位于 scripts，回归位于 tests。UE 工程、实验日志和视频不随本仓库分发。

## 历史实验资料

2026-10-07 曾完成 16 车、32 个任务的完整 UE 验收，见 [当时的复跑记录](docs/20261007_full_ue_repeat.md) 与 [停车姿态复核](docs/20261007_behavior_audit.md)。该结果属于旧版本。历史日志和演示视频分享：[百度网盘 · demo](https://pan.baidu.com/s/19O_M7CYBkYUeR5J2xVH-1w?pwd=2006)，提取码 2006；不把历史视频视为当前冻结版本的实测证据。
