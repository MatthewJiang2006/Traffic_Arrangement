#!/usr/bin/env python3
"""Summarize completed lightweight results without claiming UE physics acceptance."""
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
RUNS=ROOT/'runs/ue_transport_lightweight'
cases=['single','two','four','eight','blocked_exit']
reports={case:json.loads((RUNS/case/'acceptance.json').read_text()) for case in cases}
if any(r['status']!='PASS' for r in reports.values()):raise RuntimeError('cannot publish incomplete acceptance')
summary={'status':'UE_MAPPED_LIGHTWEIGHT_TRANSPORT_PASS','ue_physics_validated':False,
         'cases':{case:{k:v for k,v in r.items() if k not in ('task_events','traffic_events')} for case,r in reports.items()}}
(ROOT/'docs/ue_transport_lightweight.json').write_text(json.dumps(summary,indent=2)+'\n')
lines=['# 当前园区拓扑的轻量连续运输验收','',
       '结果为 **UE_MAPPED_LIGHTWEIGHT_TRANSPORT_PASS**，尚未通过 UE 物理验收。', '',
       '| 场景 | 车辆 | 完成任务 | 仿真时间（s） | 计算时间（s） | 最大路径误差（m） | 结果 |',
       '|---|---|---|---|---|---|---|']
for case,r in reports.items():
    lines.append(f"| {case} | {r['vehicle_count']} | {r['completed_tasks']}/{r['task_count']} | {r['simulation_time_s']:.1f} | {r['wall_time_s']:.2f} | {r['maximum_path_error_m']:.3f} | PASS |")
lines.extend(['','所有最终通过场景的保守车身重叠、非法受控区进入、物理道路容量超限均为零。八车场景同时活动和同时运动车辆均达到 8，两条跨区连接均被任务路线覆盖。','',
'''## 已完成的实现

使用当前 UE 实测地图生成独立运输扩展图：188 个节点、250 条边、312 个合法 Movement。原实测基础图保留。扩展图加入 A/B/C/D 站点、前进式掉头和同向宽路变道；保留场景标识及两车道直行、左车道左转、右车道右转的规则。全部 12 种站点组合均有合法往返路线，A*/Dijkstra 结果一致。

共享路线与控制器自动执行到达、2 秒虚拟测试服务、掉头、返回及后续任务。八车每辆两轮往返，共 32 个单程任务。路口和变道区域使用 FCFS 独占放行，整车离开后释放；道路分段继续使用原物理道路容量。同车道按共同道路位置保持 FIFO，不按各车无关的总行程排序。局部单调投影避免重复道路与三圆弧掉头跳段。

出口恢复场景将 D 末端准入关闭至仿真第 300 秒，排队后重新开放，8 车的 16 个任务全部完成。此前保护区不足、停车余量不足、收集点仍占控制区和 FIFO 错序的失败诊断保存在 `runs/ue_transport_lightweight/diagnostics/`。

## 模型和点位范围

轻量模型为前进式自行车运动学，保守车身长 8.344 m、宽 2.60 m，轴距 2.3285 m，转角上限 40°。步长 0.02 s，控制频率 10 Hz；目标上限直路 3 m/s、普通转弯 1.5 m/s、掉头 1 m/s，包含提前制动、跟车及停车。仿真以自己的时间推进，不等待实际秒数。

运输图位于 `config/topology/industrialarea_ue_transport.json`。A/C 截面采用 UE Y=70 m、284 m，比此前标记向园区内移动 5 m，给到达后的外向端部掉头留出空间；同向左右车道都可到达。B/D 进入末端路口后才开始转向。本轮未同步 UE 中的旧标记。

A/B/C/D 是道路测试站点，尚不是仓库。2 秒虚拟服务不代表完成装卸设施、离路泊位或等待区。服务等待与交通等待分别计时。车辆完成最后一轮往返和掉头后继续前进到受控区外的收集点，测试夹具才移出交通；主路终点不作长期停车位。

## 运行与回放

```bash
.venv/bin/python scripts/build_ue_transport_map.py
.venv/bin/logistics-park-ue-lightweight --vehicles 8 --cycles 2
.venv/bin/logistics-park-ue-lightweight --vehicles 8 --cycles 1 --block-until 300 --output runs/ue_transport_lightweight/blocked_exit
```

用浏览器打开 `runs/ue_transport_lightweight/eight/replay.html`。回放可播放、暂停、拖动时间，并选择 1/5/20/50 倍播放速度；倍率只改变回放。程序不需要启动 UE，不操控 UE 车辆或观察相机。原始任务和交通事件、轨迹、完整路线及回放位于各场景目录。摘要见 [JSON](ue_transport_lightweight.json)。

## 尚待完成

这些结果不包含 UE 轮胎接地、建筑/路缘碰撞、货厢扫掠和真实控制动力学。新的 A/C 外向端部掉头也尚未通过 UE 实测。下一步将共享路线和控制器接入正式 AirSim，同步点位，依次验证 UE 单车往返、2/4/8 车连续任务与出口堵塞恢复。

正式仓库、装卸点和离路等待区仍需结合用户场景确定。全路口所有转向覆盖、视觉闭环、冲突区时间窗与高级调度仍属后续阶段；本轮未将这些项记为完成。
'''])
(ROOT/'docs/ue_transport_lightweight.md').write_text('\n'.join(lines),encoding='utf-8')
print(summary['status'])
