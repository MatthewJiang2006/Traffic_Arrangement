# 仿真环境范围

项目原有的圆角环路、FCFS 四向路口与分支运输图继续作为 Phase 0～4 算法测试夹具；其中的道路、仓库和服务时间不是 UE 场景参数。

当前打开的 UE4 / AirSim `IndustrialArea（复件）` 道路布局已定稿。两份复制区域按用户确认合并为一个园区；已提取 [UE 道路拓扑与车辆位置](industrialarea_ue_roadgraph.md)，并将 24 辆车放到行驶车道。仓库、装卸点和任务起终点尚未指定，所以 UE 地图暂不运行任务闭环。两条跨区连接北向通行及代表性左右转已完成单车诊断，见 [UE 动态核验](ue_dynamic_acceptance.md)。Phase 11 仍需全路口、多车、相机和正式 Adapter 验收。

完整的现有算法回归入口：

```bash
.venv/bin/python scripts/check.py
```

这项检查覆盖 Python 测试夹具及 UE 拓扑的静态结构，不等于 UE 动态驾驶验收。UE 的道路标注、车辆位置和验证记录见链接文档。
