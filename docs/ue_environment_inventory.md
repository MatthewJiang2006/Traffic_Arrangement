# UE / AirSim 场景核验（2026-10-01）

当前使用 `/home/song03/Traffic_ue4/Traffic_ue4.uproject`（UE4.27.2）中打开的 `IndustrialArea（复件）`，AirSim Car RPC 为 `127.0.0.1:41451`。场景包含两份复制的 IndustrialArea；按用户确认，两份共同构成**一个园区**，左侧与中间两条纵向道路均可跨区通行。仓库、装卸点及任务终点尚未确定。

读取 UE 编辑世界与 PIE 世界后，已导出 4265 个编辑世界 Actor、24 辆 `TrafficTruckPawn`，并按道路网格和实际路面箭头生成 [道路拓扑](industrialarea_ue_roadgraph.md)。道路方向以场景标识为准。道路布局保持原样，已保存两块接缝碰撞补面和一个固定 PlayerStart。

24 辆车当前已摆在四条水平行驶车道，每条 6 辆，朝向与行驶方向一致。位置写入本机 AirSim `settings.json`，运行中的车辆也已同步摆放；[车辆位置计划](../config/traffic/industrialarea_ue_spawn_plan.json) 使用 UE 绝对世界米制坐标，精确设置和校验说明见道路拓扑文档。原始设置备份位于 `runs/ue_scene/settings_before_spawn_plan.json`。

AirSim `simGetVehiclePose()` 的坐标原点是**每辆车自己的生成点**；跨车比较或映射到道路必须使用世界坐标，并结合当前 PIE 的世界原点。该原点可能随会话变化，不能把本次的重基准偏移硬编码为长期常量。

顶视相机 `IntersectionTop` 已在 AirSim 配置中，但尚未完成相机到 RoadGraph 的标定。静态道路与车道位置、重启后的轮胎贴地、两条跨区连接北向通行及代表性左右转已核验，见 [当前短货厢行驶验收](ue_short_cargo_driving_acceptance.md)。正式 Adapter 与指定四车双路口已通过；相机标定、全路口与大规模多车通行仍待验收。

两车同车道跟车、前车停车后后车制动、重新起步恢复跟车的实测已通过，见 [双车验收](ue_following_acceptance.md)。非玩家车辆已自动绑定 AIController，24/24 车辆拥有控制器；正式 Adapter 和指定双路口调度已通过，见下文。

一个代表性路口的两车交叉让行、同向排队与出口入口净空不足等待测试通过，见 [路口验收](ue_junction_acceptance.md)。采用现有 FCFS 整路口互斥，已扩展到下述指定双路口；Conflict Zone 并行预约仍待实施。

选定四车在 j_2_1/j_3_1 两个相邻路口的下游排队、上游等待、开放后 FIFO 恢复与并行放行复测通过，见 [四车双路口验收](ue_four_truck_acceptance.md)。这是使用可控下游暂停的 Ground Truth 诊断，不代表全园区验收；正式 Adapter 后续复跑结果见下文。

正式 Adapter 已接入项目公共状态/控制接口，三项超时与退出制动检查、独立项目程序四车双路口实测均通过，见 [正式 Adapter 验收](ue_airsim_adapter.md)。测试后 24 车恢复原位且静止，144 个可见车轮贴地通过，API 控制全部释放。
