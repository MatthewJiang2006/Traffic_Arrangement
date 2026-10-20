# Demo UE 环境与工程配置

## 日常运行

| 项目 | 配置 |
|---|---|
| UE | 4.27 |
| 工程 | `/home/aipc/Traffic_ue4/Traffic_ue4.uproject` |
| 关卡 | `/Game/IndustrialArea/Map/IndustrialArea（复件）` |
| 实验入口 | Demo/run.sh |
| 默认结果 | `../Demo_runs/ue_acceptance_<时间戳>/` |
| 车辆与任务 | 24 辆场景车辆，16 辆参与运输，32 个任务 |
| 时间倍率 | 1 |

TRAFFIC_UE_PROJECT_DIR 设置工程目录，DEMO_OUTPUT_ROOT 设置结果根目录，--output 设置单次实验目录。运行时核对编辑器实际工程路径、依赖摘要和已加载原生库。

编辑器保持单实例、编辑模式，run.sh 负责 PIE 启停。鼠标兼容参数使用 -norelativemousemode。视口沿用操作者相机，收尾阶段恢复保存的位姿及编辑器设置。

## 冷启动实测环境

十轮实验的受测工程为 `/home/aipc/Traffic_ue4_agent/Traffic_ue4.uproject`，每轮恢复同一工程、AirSim 和 Epic 配置基线，创建独立编辑器进程，通过 run.sh 执行实验，保存结果后关闭编辑器。

实测结果为 9 次通过、1 次等待环停滞失败，详见 [验证记录](VALIDATION_20261020.md)。日常运行与冷启动实测的工程路径分别记录，便于按环境复核结果。

## 数据与源码

Demo 源文件采用只读保护，实验数据写入外部目录。本机完整快照及测试日志位于 `../Demo_runs/replacement_backup_20261020_215051/`。实验数据和视频下载入口见 [README](../README.md)。
