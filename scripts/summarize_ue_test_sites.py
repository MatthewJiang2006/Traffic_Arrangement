#!/usr/bin/env python3
"""Publish successful four-site/representative two-truck acceptance snapshots."""
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'runs/ue_scene/test_sites'
labels=['A_single','B_single','C_single','D_single','A_two','B_two']
cases=[];paths={}
for label in labels:
 d=json.loads((OUT/label/'acceptance.json').read_text())
 if d['status']!='PASS':raise RuntimeError(label+' has not passed')
 cases.append({k:d[k] for k in ['case','status','finished','cleared','maximum_simultaneous_junction_occupancy','stationary_wait_s','max_rear_path_error_m','close_result']})
 if label.endswith('_single'):paths[label[0]]=d['path']
restored=json.loads((OUT/'latest_acceptance.json').read_text())
if restored['status']!='PASS' or not restored['fleet_restored']:raise RuntimeError('latest suite/restore has not passed')
report={'status':'PASS','cases':cases,'placement':json.loads((OUT/'placement.json').read_text()),'fleet_restored':True,'settings_unchanged':restored['settings_unchanged'],'camera_changed':False,'regression':{'pytest_passed':81,'phase_0_4':'PASS','log':'runs/ue_scene/test_sites/project_check.log'},'scope':'four single-truck U-turns; representative wide-road A and horizontal-road B two-truck FCFS checks; fixture paths not integrated as production RoadGraph movements'}
(ROOT/'docs/ue_test_sites_acceptance.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
(ROOT/'config/traffic/industrialarea_ue_uturn_paths.json').write_text(json.dumps({'coordinate_frame':'UE absolute world XY meters','path_reference':'rear axle','status':'validated_fixture_paths','paths':paths},indent=2)+'\n')
p=ROOT/'config/traffic/industrialarea_ue_test_sites.json';config=json.loads(p.read_text());config['status']='pie_markers_placed_four_single_uturns_passed';config['terminal_uturn']['implementation_status']='validated_fixture_paths_and_representative_two_truck_fcfs_not_production_graph_movements';config['validation_report']='docs/ue_test_sites_acceptance.json';p.write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n')
rows='\n'.join('| '+c['case']+' | PASS | '+str(len(c['finished']))+' | '+f"{c['max_rear_path_error_m']:.3f}"+' | '+f"{c['stationary_wait_s']:.1f}"+' |' for c in cases)
text='''# A/B/C/D 点位与掉头验收（2026-10-01）

四处起终点已在当前 PIE 布置标记，8 个点的路面高度均为 UE 绝对 1.10 m。A/C 位于四车道宽路两端，B/D 位于两份园区远离宽路的横路末端。见 [点位图](ue_test_sites.svg)、[点位配置](../config/traffic/industrialarea_ue_test_sites.json)。蓝色圆圈是出发点，橙色圆圈是到达点；图中 UE +X 向右、+Y 向上，与当前仿真窗口的视角无关。

四处单车掉头，以及宽路 A、横路 B 两组代表性双车等待/恢复通行均通过实际 UE 复测。车辆全程前进，不倒车、不在行驶中瞬移。

| 场景 | 结果 | 完成车辆 | 最大后轴路径误差（m） | 静止等待（s） |
|---|---|---|---|---|
'''+rows+'''

B/D 从到达车道沿 +X 直行进入末端路口，达到转弯起点后才允许转向，掉头后沿 −X 回到该处出发车道。控制器已明确禁止提前转向。A/C 先从同向右车道变入左车道，再在端部路口掉头，驶入反向右车道。最终选用后轴半径 3.8 m 的路径。

双车使用现有 FCFS 整路口独占放行，车身包围盒全部离开后释放，出口未留足空间时继续等待。两组测试的最大同时路口占用均为 1。前车通过到达点后继续前进 12 m，给后车留下出口及停车空间。等待时间不计入行驶停滞计时。

每轮通过正式 AirSim Adapter 控制，检查源状态新鲜度、原生碰撞、可见车身扫掠、其他车辆重叠、六个可见车轮接地与姿态。最终通过轮次没有这些检查的违规。车辆设置未改变，测试车显式恢复到运行前位置并清零运动速度；其余车辆保持位置。没有修改或锁定相机和编辑器视口。

前期小半径路径跟踪超限、大半径路径压路缘、一次状态冻结保护停车及等待恢复计时错误均已保留失败记录，最终结果单独记录。未放宽状态/命令超时和轮胎接地阈值。全车 AirSim reset 曾导致物理状态异常及越界后的对象查询崩溃；本夹具已改为只挪动测试车，显式恢复位置与速度，不再调用全车 reset。

原始结果位于 `runs/ue_scene/test_sites/<场景>/acceptance.json`、对应 `adapter.jsonl` 与 UE 车身监测历史；摘要见 [验收 JSON](ue_test_sites_acceptance.json)。81 项 pytest 与 Phase 0～4 全部闭环检查通过，日志为 `runs/ue_scene/test_sites/project_check.log`。

## 复现与范围

要求当前 PIE 运行、24 车静止，系统 Python 已有 AirSim SDK，项目后端可用。在仓库根目录执行：

```bash
/usr/bin/python3 - <<'PYTHON'
import sys
from pathlib import Path
sys.path.insert(0, 'scripts')
import airsim
from ue_trailer_driving_check import ue_script
ue_script(airsim.CarClient(), 'place_test_sites.py', Path.cwd()/'runs/ue_scene/test_sites/placement.json')
PYTHON
/usr/bin/python3 scripts/validate_ue_test_sites.py --single-only
/usr/bin/python3 scripts/validate_ue_test_sites.py --site A --site B --two-only
```

点位是当前 PIE 的调试标记，未保存进 UE 地图；重启后可按上面命令重新布置。可用自己的视角观察测试车辆 Car_0_0 / Car_0_1；脚本不会自动切换视角。若仿真状态停止更新，安全保护会停车。

验证范围为上述指定路径。C/D 双车尚未单独实测，8 车连续运行尚未开始。掉头路径已保存到 `config/traffic/industrialarea_ue_uturn_paths.json`，当前作为验收夹具的独占通行连接使用；尚未增加到正式 RoadGraph Movement 或接入通用路由、连续任务分发。到达点也不作为长期停车位。下一阶段需将这些路径、宽路变道及点位纳入正式路线执行，再扩展到 8 车。
'''
(ROOT/'docs/ue_test_sites_acceptance.md').write_text(text)
p=ROOT/'docs/ue_test_sites.md';t=p.read_text();t=t.replace('尚未在 UE 放置标记、设置车辆或完成新点位行驶验收；实际高度与附近车辆需在布置时检查。','已在当前 PIE 放置标记并完成四处单车掉头验收、宽路与横路各一组双车验收，见 [验收记录](ue_test_sites_acceptance.md) 与 [点位图](ue_test_sites.svg)。标记未保存进 UE 地图，重启后需重新布置。');t=t.replace('此项目前记录为规则授权，现有拓扑尚未增加掉头路径，转弯半径、车身扫掠和接地须实测后再接入连续运行。','指定掉头路径的半径、车身扫掠和接地已实测通过；路径目前用于验收夹具，现有正式 RoadGraph 尚未增加掉头 Movement，连续运行待下一阶段接入。');p.write_text(t)
p=ROOT/'docs/progress.md';t=p.read_text().replace('当前 78 项测试','当前 81 项测试');p.write_text(t)
p=ROOT/'README.md';t=p.read_text();addition='\nA/B/C/D 道路测试点已布置，四处单车掉头与宽路/横路两组双车独占放行实测通过。横路先直行进入末端路口再掉头；宽路先进入左车道。见 [点位图](docs/ue_test_sites.svg) 和 [点位与掉头验收](docs/ue_test_sites_acceptance.md)。\n'
if addition.strip() not in t:p.write_text(t+addition)
print('Four-site acceptance snapshots written.')
