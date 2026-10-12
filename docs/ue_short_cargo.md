# 短货厢替换

已为当前 24 辆车固定安装 `SM_Truck_Trailer_1`（约 4.314 × 2.086 × 2.487 米），开口朝向驾驶室（模型开口在负 X 端，相对车头旋转 180°）。驾驶室与货厢间距 20 厘米，货厢底部支撑与 TruckAtt 挂钩高度对齐（相比原包围盒定位下调约 69 厘米），厢底与轮胎顶部间隙约 10 厘米，货厢后缘位于底盘支承范围内。

安装逻辑已写入 TrafficTruckPawn 并编译、重启验证，每次启动自动安装。24 辆安装位置检查通过，144 个外观轮胎接地检查通过（约 -0.67 至 +0.78 厘米）。车辆初始位置与原驾驶控制保留。

本次替换属于现有 AirSim 物理底盘上的外观组合，货厢没有独立质量或物理碰撞。短货厢已完成代表性单车直行、左右转、停车和保守包络碰撞核验，见 [单车行驶验收](ue_short_cargo_driving_acceptance.md)。货厢物理动力学和全园区、多车通行仍未验收。

原始检查：`runs/ue_scene/short_cargo_validation.json`、`runs/ue_scene/truck_contact_validation.json`；旧源代码与配置备份：`runs/ue_scene/short_cargo_source_backup/`。
