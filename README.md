# Factory Traffic Management · 园区运输调度

本模块包含园区物流运输的 Python 调度算法、道路与服务点拓扑、车辆控制、AirSim 接口、轻量仿真和 UE 验收脚本。车辆连续执行装货、运输、卸货和等待位停车任务，运行时检查通行许可、道路容量、跟车间距、车体接触和局部停滞。

仓库其他模块与整体结构见 [根目录 README](../../../Readme.md)。

## 模块结构

```text
Factory_Traffic_Management_script/
├── src/logistics_park/
│   ├── common/          # 公共消息与数据模型
│   ├── map/             # 道路图与几何
│   ├── navigation/      # 定位匹配与路径跟踪
│   ├── routing/         # 服务节点路由
│   ├── scheduling/      # 任务分派、资源预约与通行时序
│   ├── tasks/           # 任务状态与生命周期
│   ├── runtime/         # 连续任务、车辆控制与运行检查
│   ├── safety/          # 车体范围与碰撞检查
│   ├── adapters/        # 仿真接口
│   └── simulation/      # 轻量仿真
├── config/              # 拓扑、泊位、任务、交通策略和运行参数
├── scripts/             # 构图、预检、真实 UE 验收与结果复核
├── tests/               # 回归测试和独立测试夹具
├── docs/                # 设计、路网、接口和修复记录
├── runs/                # 本地运行输出，分享数据见百度网盘
├── 项目规划.md
├── 调度算法优化方案.md
└── README.md
```

## 最近完整 UE 验收

2026-10-07 的完整验收 **PASS**：16 辆车完成 32/32 个运输任务并全部停入等待位，正常时间倍率 1，用时 1725.769 秒（约 28 分 46 秒）。

- 车辆碰撞、车体障碍接触、未授权通行、道路容量违规和停滞报警均为 0；任务期间瞬移为 0。
- 最终停车最大朝向偏差 2.147°，最大位置偏差 0.264 米，满足 5° 和 0.35 米的验收阈值。
- 直线路段整车范围检查覆盖 67,838 个样本，越界标记为 0，最小边界余量约 0.204 米。
- 全套 539 项回归测试和完整轻量预检通过；车辆、API、地图与编辑器状态恢复检查通过。

直线范围检查限于整车已进入直线路段的样本；转弯和接入口由运行中的车身扫描、碰撞和路线误差检查覆盖。停车控制、改道许可、出发准入和排队顺序的修复过程见 [完整复跑记录](docs/20261007_full_ue_repeat.md) 与 [停车姿态复核](docs/20261007_behavior_audit.md)。

对应配置为 `config/runtime/industrialarea_ue_full_verified_20261007.json`。

## 实验数据与视频下载

实验数据、原始日志、验收报告和演示视频在百度网盘分享。视频及原始日志体积较大，无法直接上传 GitHub，请从网盘下载。

- 分享文件：**demo**
- 下载链接：[百度网盘 · demo](https://pan.baidu.com/s/19O_M7CYBkYUeR5J2xVH-1w?pwd=2006)
- 提取码：**2006**

结果入口为 `acceptance_summary.json`；详细报告为 `physical_acceptance.json`，轨迹与控制日志可用于复核。完整录屏 `ue_full_demo.mp4` 约 28 分 52 秒，16 倍速视频 `ue_full_demo_16x.mp4` 约 1 分 48 秒。

## 离线运行与检查

以下命令在本模块目录执行，使用 Python 3.10 或更高版本：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install pytest

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src python -m pytest -p no:nose -q
```

完整轻量任务预检：

```bash
python scripts/simulate_ue_continuous_tasks.py \
  --scenario config/runtime/industrialarea_ue_full_verified_20261007.json
```

其他阶段的测试与轻量验收：

```bash
PYTHONPATH=src python scripts/check.py
```

轻量仿真使用 Python 标准库，生成的报告与日志写入本地 `runs/`，工作目录会由脚本创建。

## 真实 UE 验收

需要单独配置 UE4.27 场景、AirSim 插件及匹配的 Python 客户端、RPC 连接和 UE Python 远程执行。本仓库提供算法和验证脚本，UE 工程需另行准备。运行前应按本机环境设置脚本中的 UE 工程与插件路径，并确认配置中的地图指纹与当前地图一致。

完成对应轻量预检，并让 UE 场景进入可连接的运行状态后，使用：

```bash
python scripts/validate_ue_continuous_tasks.py \
  --scenario config/runtime/industrialarea_ue_full_verified_20261007.json \
  --time-scale 1
```

UE 远程执行工具为 `scripts/ue_remote.py`，真实连续任务入口为 `scripts/validate_ue_continuous_tasks.py`。结果复核工具为 `scripts/audit_ue_run.py` 与 `scripts/audit_ue_road_envelope.py`。

## 设计文档

- [服务节点路网](docs/ue_service_network.md)
- [动态车队接口](docs/dynamic_fleet_interfaces.md)
- [任务调度](docs/task_dispatch.md)
- [早期架构约定](docs/architecture.md)
- [项目规划](项目规划.md)
- [调度算法优化方案](调度算法优化方案.md)
