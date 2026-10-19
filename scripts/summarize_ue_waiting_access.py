#!/usr/bin/env python3
"""Publish measured remaining waiting-berth access and service clearance."""
import json
from pathlib import Path
from validate_ue_loading_access import map_fingerprints
from ue_access_evidence import unchanged_access_region

ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/"runs/ue_scene/station_siting"
SITES=("Q1","Q2","Q6","Q7","Q8","Q9","Q10","Q11","Q12")
LANES=("LOWER-north","LOWER-south","UPPER-north","UPPER-south")
current_maps=map_fingerprints()


def evidence(site):
    path=RAW/f"waiting_access_{site}.json"
    source=json.loads(path.read_text()) if path.exists() else {}
    current=source.get("map_sha256")==current_maps
    preserved=unchanged_access_region(source,path.stem,current_maps)
    status=source.get("status","PENDING")
    if status=="PASS" and not (current or preserved):
        status="STALE_MAP"
    phases={}
    for name,phase in source.get("phases",{}).items():
        samples=phase["samples"]
        phases[name]=dict(status=phase["status"],reverse=phase["reverse"],
                         max_path_error_m=max((s["path_error_m"] for s in samples),default=None),
                         max_speed_m_s=max((s["speed_m_s"] for s in samples),default=None),
                         body_collision_samples=sum(any(s["body_hits"]) for s in samples),
                         vehicle_overlap_samples=sum(any(s["parked_vehicle_overlaps"]) for s in samples),
                         obstacle_contact_samples=sum(bool(s["collision"]) and s.get("collision_classification")!="ground_surface_contact" for s in samples),
                         stationary_hold_s=phase.get("stationary_hold_s",0))
    passed=bool(status=="PASS" and source.get("vehicle_restored") and phases and
                all(p["status"]=="PASS" and not p["body_collision_samples"] and
                    not p["vehicle_overlap_samples"] and not p["obstacle_contact_samples"] for p in phases.values()))
    return dict(id=site,status=status,continuous_entry_exit_pass=passed,phases=phases,
                current_saved_map_verified=current,access_clearance_preserved=preserved,
                routing_enabled=False,evidence=str(path.relative_to(ROOT))),source


rows=[]
routes={}
for site in SITES:
    row,raw=evidence(site)
    rows.append(row)
    if row["continuous_entry_exit_pass"]:
        routes[site]={name:dict(path_xy_heading=p["path_xy_heading"],reverse=p["reverse"])
                      for name,p in raw["phases"].items()}
lanes={site:evidence(site)[0] for site in LANES}
clearance=evidence("W5-waiting-clearance")[0]
summary=dict(status="PASS" if all(r["continuous_entry_exit_pass"] for r in rows)
             and all(r["continuous_entry_exit_pass"] for r in lanes.values())
             and clearance["continuous_entry_exit_pass"] else "PENDING",sites=rows,
             adjacent_lane_cases=lanes,w5_with_q1_q2_occupied=clearance,
             map_sha256=current_maps,wheel_contact_is_diagnostic=True,routing_enabled=False,
             queue_rules={"Q1_departure_requires_Q2_empty":True,
                          "Q1_Q2_access_corridor_exclusive":True,
                          "Q6_Q7_Q8_shared_entry_area_requires_reservation":True,
                          "Q9_Q10_shared_entry_area_requires_reservation":True,
                          "Q11_Q12_shared_entry_area_requires_reservation":True})
(ROOT/"docs/ue_waiting_access_acceptance.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n")
lines=["# 共享等待位道路进出验收","",
       "本轮范围为 Q1/Q2、Q6～Q12。验收以完整跑通、车身无障碍接触、无车辆碰撞为准；轮胎轻微离地只记录。开场清空附近车辆，在实际行驶车道摆车，阶段之间不传送，等待位停稳保持 3 秒，完成后复位车队。相机保持自由。本轮提速复测的入口直线段采用 2 m/s 上限、转弯 1.5 m/s、接近终点制动，四段直行回归采用 3 m/s 上限（约 10.8 km/h）；未受施工影响的旧证据仍保留原验收速度。","",
       "| 等待位 | 连续道路进出 | 停稳等待 | 车身碰障碍 | 车辆重叠 |","|---|---|---:|---:|---:|"]
for r in rows:
    hold=r["phases"].get("waiting_entry",{}).get("stationary_hold_s",0)
    lines.append(f"| {r['id']} | {r['status']} | {hold} s | {sum(p['body_collision_samples'] for p in r['phases'].values())} | {sum(p['vehicle_overlap_samples'] for p in r['phases'].values())} |")
lines += ["","| 入口邻接车道直行 | 状态 |","|---|---|"]
for site,r in lanes.items():
    lines.append(f"| {'低 Y' if site.startswith('LOWER') else '高 Y'} 中间纵路{'北行' if site.endswith('north') else '南行'} | {r['status']} |")
lines += ["",f"Q1/Q2 同时停放时，从中间纵路到 W5 两个装卸位的作业通道检查：**{clearance['status']}**。该测试停放两辆等待车，再让作业车驶入 W5、停稳、驶离，检查整车重叠。","",
          "Q1/Q2 与 W5 共用一条硬化通道。Q2 有车时，Q1 沿 +X 前往 W5 的路线会穿过 Q2，必须先放行 Q2，禁止同时进出；两处等待车对从东侧纵路进出的 W5 作业是否有影响，以独立三车实测为准。Q6～Q8 邻近入口区域也需预约后才能进行多车作业。","",
          "本轮清除 Q6/Q8 转弯范围内四段小围挡，保留大型建筑；Q9/Q10、Q11/Q12 各加两段共享路缘坡道，另移除 Q10/Q12 前进路线上的两段小围栏，改为前进驶入；Q12 向北出位曾在坡道交界卡住，最终采用入口曲线倒车退出后恢复北行，原向北出位方案弃用；两份地图修改前已备份。施工记录见 `runs/ue_scene/station_siting/remaining_waiting_access_patch.json`。 后续已将两处横路接缝坡道延长至 4.8 m，搭接硬化区域并消除原约 17 cm 高差；Q9～Q12 和四段邻接车道已在当前地图复测通过，见 [坡道修复验收](ue_seam_repair_acceptance.md)。未受修复影响的旧证据另由 `access_regions_after_ramp_seam_repair.json` 核对路线周围静态几何。","",
          "未修改原装卸试验范围内的静态网格、位置、旋转和包络，前后快照核对见 `previous_access_regions_after_waiting_patch.json`，据此保留相应历史实车结果。其余既有等待位通过全场静态快照核对，确认只有这两段围栏被移除，其他网格、位置和碰撞包络均未改变；记录见 `waiting_fence_removal_preservation.json`。新点位仍未启用自动调度；下一阶段接入泊位/通道预约与运输任务状态机。"]
(ROOT/"docs/ue_waiting_access_acceptance.md").write_text("\n".join(lines)+"\n")
if summary["status"]=="PASS":
    (ROOT/"config/planning/ue_waiting_access_routes.json").write_text(json.dumps(dict(
        status="VALIDATED_LOW_SPEED_ACCESS",map_sha256=current_maps,routing_enabled=False,
        sites=routes,queue_rules=summary["queue_rules"]),ensure_ascii=False,indent=2)+"\n")
print(summary["status"],[(r["id"],r["status"]) for r in rows])
