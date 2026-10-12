# IndustrialArea UE 道路拓扑与车辆位置

两份复制区域合并为一座园区，共 12 个路口、16 段路口间走廊。左、中纵路连接两区，右侧接缝没有道路连接。当前拓扑为 100 个节点、136 条有向边、172 个 Movement，见 [地图](industrialarea_ue_roadgraph.svg)、[RoadGraph](../config/topology/industrialarea_ue.json) 与 [测绘记录](../config/topology/industrialarea_ue_survey.json)。

左右以车辆自身朝向为准。UE 车头局部 +X 向前、+Y 向右，正 yaw 为右转；行驶方向的右侧单位向量为 `(-sin(yaw), cos(yaw))`。图中的 E/W/N/S 仅为 UE +X/−X/+Y/−Y 的端口代号，不是车辆左右。`Mat_RoadMarking_1` 箭头朝向为 `-decal_right_vector`，已用俯视画面核对；共识别 10 个直行加左转箭头。

左侧宽纵路每方向两条车道，中心相对道路中心向车辆右侧偏移 1.9 米和 5.7 米。左车道允许直行、左转，右车道允许直行、右转；直行保持车道。其他道路每方向一条车道，使用 SINGLE 标签，有明确直行加左转箭头的入口不生成右转连接。道路模型包围盒包含人行道，不能据此把每条道路都扩成双车道；[横断面射线记录](../runs/ue_scene/lane_surface_probe.json) 区分约 1.10 米沥青面和 1.25 米人行道。

两条跨区道路原有约 0.62 米接缝，现保存不可见静态碰撞补面。24 辆车保持校准后的位置与高度，朝向按实际路面箭头修正并写入 AirSim 设置：UE +X 方向在道路中心 +Y 一侧，−X 方向在 −Y 一侧。车辆 Actor Z 约 0.894 米；外观轮胎接地另行验证。隐藏 SUV 底盘绑定正确的 WheelFL/FR/BL/BR 物理轮骨骼，BP_Truck 外观相对 Z 为 +21 厘米。

车辆已固定安装短货厢 [SM_Truck_Trailer_1](ue_short_cargo.md)，开口朝车头，底部支撑与 TruckAtt 挂钩高度对齐。单车实际驾驶结果见 [行驶验收](ue_short_cargo_driving_acceptance.md)。货厢为外观组件，没有独立物理质量或碰撞；驾驶验证同时查询保守车身包络。

在运行的 PIE 中执行 `scripts/ue/export_scene.py`，再运行 `scripts/build_ue_roadgraph.py` 可重建拓扑和车辆位置快照。`industrialarea_ue_spawns_initial.json` 保留摆放前历史状态；当前状态为 `industrialarea_ue_spawns.json`。仓库、装卸点和任务起终点等待用户确定。[正式 Adapter 与选定四车双路口](ue_airsim_adapter.md) 已验收；全路口通行、大规模多车交通和视觉闭环仍待实施，当前不代表 Phase 11 整体验收完成。
