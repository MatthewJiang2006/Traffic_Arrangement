#!/usr/bin/env python3
"""Publish evidence for six service bays without enabling automatic dispatch."""
import hashlib
import json
from pathlib import Path
from ue_access_evidence import unchanged_access_region

ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/"runs/ue_scene/station_siting"
MAPS=Path("/home/song03/Traffic_ue4/Content/IndustrialArea/Map")
fingerprints={n:hashlib.sha256((MAPS/n).read_bytes()).hexdigest()
              for n in ("IndustrialArea（复件）.umap","IndustrialArea_Copy.umap")}
sites=("W1-unloading","W2-unloading","W5-loading","W5-unloading","W6-loading","W6-unloading")
rows=[]
routes={}
for site in sites:
    path=RAW/f"service_access_{site}.json"
    raw=json.loads(path.read_text()) if path.exists() else {}
    phases={}
    for name,phase in raw.get("phases",{}).items():
        samples=phase["samples"]
        phases[name]=dict(status=phase["status"],reverse=phase["reverse"],
                         duration_s=samples[-1]["t_s"] if samples else 0,
                         max_path_error_m=max((s["path_error_m"] for s in samples),default=None),
                         body_collision_samples=sum(any(s["body_hits"]) for s in samples),
                         vehicle_overlap_samples=sum(any(s["parked_vehicle_overlaps"]) for s in samples),
                         obstacle_contact_samples=sum(bool(s["collision"]) and
                             s.get("collision_classification")!="ground_surface_contact" for s in samples),
                         max_speed_m_s=max((s["speed_m_s"] for s in samples),default=0))
    current=raw.get("map_sha256")==fingerprints
    preserved=unchanged_access_region(raw,path.stem,fingerprints)
    status=raw.get("status","PENDING")
    if status=="PASS" and not (current or preserved):
        status="STALE_MAP"
    passed=(status=="PASS" and raw.get("vehicle_restored",False) and phases
            and all(p["status"]=="PASS" and p["body_collision_samples"]==0
                    and p["vehicle_overlap_samples"]==0 and p["obstacle_contact_samples"]==0
                    for p in phases.values()))
    rows.append(dict(id=site,status=status,continuous_entry_exit_pass=bool(passed),
                     current_saved_map_verified=current,unchanged_region_verified=preserved,phases=phases,
                     vehicle_restored=raw.get("vehicle_restored",False),
                     routing_enabled=False,evidence=str(path.relative_to(ROOT))))
    if passed:
        routes[site]={name:dict(path_xy_heading=p["path_xy_heading"],reverse=p["reverse"])
                      for name,p in raw["phases"].items()}
lanes={}
for lane in ("MIDDLE-north","MIDDLE-south"):
    path=RAW/f"service_access_{lane}.json"
    raw=json.loads(path.read_text()) if path.exists() else {}
    samples=raw.get("phases",{}).get("straight",{}).get("samples",[])
    current=raw.get("map_sha256")==fingerprints
    preserved=unchanged_access_region(raw,path.stem,fingerprints)
    status=raw.get("status","PENDING")
    if status=="PASS" and not (current or preserved):
        status="STALE_MAP"
    lanes[lane]=dict(status=status,current_saved_map_verified=current,unchanged_region_verified=preserved,
                     vehicle_restored=raw.get("vehicle_restored",False),
                     body_collision_samples=sum(any(s["body_hits"]) for s in samples),
                     vehicle_overlap_samples=sum(any(s["parked_vehicle_overlaps"]) for s in samples),
                     evidence=str(path.relative_to(ROOT)))
q1_path=RAW/"service_access_W5-from-Q1.json"
q1_raw=json.loads(q1_path.read_text()) if q1_path.exists() else {}
q1_current=q1_raw.get("map_sha256")==fingerprints
q1_preserved=unchanged_access_region(q1_raw,q1_path.stem,fingerprints)
q1_status=q1_raw.get("status","PENDING")
if q1_status=="PASS" and not (q1_current or q1_preserved):
    q1_status="STALE_MAP"
q1=dict(status=q1_status,current_saved_map_verified=q1_current,unchanged_region_verified=q1_preserved,
        vehicle_restored=q1_raw.get("vehicle_restored",False),evidence=str(q1_path.relative_to(ROOT)),
        scope="Forward from an already occupied Q1 waiting berth to W5 loading/unloading and the northbound road")
summary=dict(status="PASS" if all(r["continuous_entry_exit_pass"] for r in rows)
             and all(r["status"]=="PASS" and r["vehicle_restored"] for r in lanes.values())
             and q1["status"]=="PASS" and q1["vehicle_restored"] else "PENDING",
             map_sha256=fingerprints,routing_enabled=False,wheel_contact_is_diagnostic=True,
             scope="Continuous single-truck low-speed approach, berth and road recovery",sites=rows,
             adjacent_lane_cases=lanes,w5_forward_from_q1=q1)
(ROOT/"docs/ue_remaining_service_acceptance.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n")
lines=["# 剩余仓库装卸位进出验收","",
       "验收以完整跑通、车身不触碰障碍、无车辆碰撞为准，轮胎轻微离地仅记录。测试开场清空附近车辆并在合法车道摆车，阶段之间不传送；完成后恢复全部车辆，镜头保持自由。","",
       "| 点位 | 进出方式 | 连续实车状态 | 车身碰障碍 | 车辆重叠 |",
       "|---|---|---|---:|---:|"]
for r in rows:
    mode="前进入位、倒车退出、恢复前进" if r["id"].startswith(("W1","W2")) else "倒车入位、前进驶出"
    lines.append(f"| {r['id']} | {mode} | {r['status']} | {sum(p['body_collision_samples'] for p in r['phases'].values())} | {sum(p['vehicle_overlap_samples'] for p in r['phases'].values())} |")
lines += ["","| 中间纵路回归 | 状态 |","|---|---|"]
for lane,r in lanes.items():
    lines.append(f"| {'北行车道' if lane.endswith('north') else '南行车道'} | {r['status']} |")
lines += ["",f"Q1 → W5 前进方案：**{q1_status}**。车辆从 Q1 等待位置沿 +X 前进，分别在 W5 装货位、卸货位停稳，再经入口坡道前进回到中间纵路北行车道。Q1/Q2 与通道需先清空；Q1/Q2 的完整道路接入及等待车辆对 W5 的影响见[等待位进出验收](ue_waiting_access_acceptance.md)。"]
if any(r["unchanged_region_verified"] for r in rows):
    lines += ["", "本轮等待位施工后，原装卸试验范围内的静态网格、位置、旋转和包络经前后快照核对未改变，保留历史实车结果，并单列 `unchanged_region_verified`。证据见 `runs/ue_scene/station_siting/previous_access_regions_after_waiting_patch.json`。"]
lines += ["","W1/W2 卸货位前方为大型建筑，保留建筑并沿入位通道倒车退出；在道路端点换回前进挡，按西向车道驶离。W5/W6 四点从中间纵路北行车道前进到倒车准备位，经半径 6 m 的圆弧和直线倒入泊位，再前进回到同一北行车道。路线均核对定稿车道方向。","",
          "W1/W2 卸货入口和 W5/W6 入口实车发现路缘高差会卡车，已各补两段局部坡道，两份地图均有修改前备份，记录见 `runs/ue_scene/station_siting/middle_service_ramps.json`、`unloading_service_ramps.json`；没有拆除物体。测试期间其他车辆静止，已清空作业范围；这属于单车进出验证，尚未覆盖泊位预约、双向动态会车、运营速度或装卸服务调度，自动调度仍未启用。","",
          "地图校验值及逐阶段指标见 `ue_remaining_service_acceptance.json`；原始采样见 `runs/ue_scene/station_siting/service_access_*.json`。"]
(ROOT/"docs/ue_remaining_service_acceptance.md").write_text("\n".join(lines)+"\n")
if summary["status"]=="PASS":
    (ROOT/"config/planning/ue_remaining_service_routes.json").write_text(json.dumps(dict(
        status="VALIDATED_SINGLE_TRUCK_LOW_SPEED",map_sha256=fingerprints,
        station_plan_sha256=hashlib.sha256((ROOT/"config/planning/ue_station_proposal.json").read_bytes()).hexdigest(),
        routing_enabled=False,sites=routes,
        alternatives={"W5_from_Q1":{name:dict(path_xy_heading=p["path_xy_heading"],reverse=p["reverse"])
                                   for name,p in q1_raw["phases"].items()}}),ensure_ascii=False,indent=2)+"\n")
print(summary["status"],[(r["id"],r["status"]) for r in rows])
