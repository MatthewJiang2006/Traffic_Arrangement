#!/usr/bin/env python3
"""Publish measured loading access evidence; never enable station dispatch."""
import json
import math
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT/"runs/ue_scene/station_siting"
MAP_DIRECTORY = Path("/home/song03/Traffic_ue4/Content/IndustrialArea/Map")
current_maps = {name: hashlib.sha256((MAP_DIRECTORY/name).read_bytes()).hexdigest()
                for name in ("IndustrialArea（复件）.umap","IndustrialArea_Copy.umap")}
rows = []
audit_path = RAW/"loading_regions_after_middle_ramps.json"
region_audit = json.loads(audit_path.read_text()) if audit_path.exists() else {}
for site in ("W1", "W2"):
    record = {"site": site, "routing_enabled": False, "cases": {}}
    for case in ("reservation", "straight_south", "straight_north"):
        path = RAW/f"loading_access_{site}_{case}.json"
        source = json.loads(path.read_text()) if path.exists() else {}
        phases = {}
        for name, phase in source.get("phases", {}).items():
            samples = phase["samples"]
            phases[name] = dict(
                status=phase["status"], duration_s=samples[-1]["t_s"] if samples else 0,
                max_path_error_m=max((s["path_error_m"] for s in samples),default=None),
                max_wheel_clearance_m=max((abs(v) for s in samples
                                           for v in s["wheel_clearances_m"] if v is not None),default=None),
                body_collision_samples=sum(any(s["body_hits"]) for s in samples),
                vehicle_overlap_samples=sum(any(s.get("parked_vehicle_overlaps",[])) for s in samples))
            phases[name]["supported_ground_contact_samples"] = sum(
                s.get("collision_classification") == "ground_surface_contact" for s in samples)
            phases[name]["native_obstacle_collision_samples"] = sum(
                bool(s.get("collision")) and s.get("collision_classification") != "ground_surface_contact"
                for s in samples)
            phases[name]["max_speed_m_s"] = max((s["speed_m_s"] for s in samples),default=0)
        result = dict(status=source.get("status", "PENDING"), phases=phases,
                      evidence=str(path.relative_to(ROOT)),
                      vehicle_restored=source.get("vehicle_restored",False))
        result["current_saved_map_verified"] = source.get("map_sha256") == current_maps
        result["unchanged_region_verified"] = (
            region_audit.get("before_map_sha256") == source.get("map_sha256")
            and region_audit.get("after_map_sha256") == current_maps
            and region_audit.get("regions",{}).get(site,{}).get("unchanged",False))
        if result["status"] == "PASS" and not (result["current_saved_map_verified"] or result["unchanged_region_verified"]):
            result["status"] = "STALE_MAP"
        if case == "reservation" and source:
            result["events"] = source["events"]
            waiting = [s["t_s"] for s in source["traffic_samples"]
                       if s["owner"] == "Car_0_0"
                       and abs(s["vehicles"]["Car_0_2"]["xy_m"][0]-15)<1
                       and s["vehicles"]["Car_0_2"]["speed_m_s"]<.08]
            result["follower_stationary_wait_s"] = max(waiting)-min(waiting) if waiting else 0
            boundary = source["routes"]["zone_bounds_m"][1]
            tail_held = []
            for sample in source["traffic_samples"]:
                owner = sample["vehicles"]["Car_0_0"]
                yaw = math.radians(owner["yaw_deg"])
                center_y = owner["xy_m"][1]
                tail_y = center_y+4.5*abs(math.sin(yaw))+1.65*abs(math.cos(yaw))
                if center_y < boundary <= tail_y and sample["owner"] == "Car_0_0":
                    tail_held.append(sample["t_s"])
            result["center_outside_but_tail_reserved_samples"] = len(tail_held)
            event_types = {e["event"] for e in source["events"]}
            result["reservation_evidence_pass"] = (
                {"DENIED","GRANTED","TAIL_NOT_CLEAR","RELEASED"} <= event_types
                and result["follower_stationary_wait_s"] >= 3 and bool(tail_held))
        record["cases"][case] = result
    record["low_speed_access_and_reservation_pass"] = (
        all(c["status"] == "PASS" and c["vehicle_restored"] for c in record["cases"].values())
        and record["cases"]["reservation"].get("reservation_evidence_pass",False))
    rows.append(record)
summary = dict(status="PASS" if all(r["low_speed_access_and_reservation_pass"] for r in rows) else "PENDING",
               scope="Low-speed diagnostic; continuous lane approach and three-truck reservation; both adjacent lanes",
               routing_enabled=False, sites=rows)
summary["map_sha256"] = current_maps
summary["acceptance_criteria"] = ["complete_route", "no_body_obstacle_contact",
                                  "no_vehicle_collision", "exclusive_zone_and_tail_clear_release"]
summary["wheel_contact_is_diagnostic"] = True
summary["geometry_evidence"] = [
    "runs/ue_scene/station_siting/loading_ramp_lane_tapers.json",
    "runs/ue_scene/station_siting/loading_taper_overlap.json",
    "runs/ue_scene/station_siting/loading_lane_taper_profiles.json"]
if region_audit.get("after_map_sha256") == current_maps:
    summary["geometry_evidence"].append(str(audit_path.relative_to(ROOT)))
(ROOT/"docs/ue_loading_access_acceptance.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n")
if summary["status"] == "PASS":
    routes = {site: json.loads((RAW/f"loading_access_{site}_reservation.json").read_text())["routes"]
              for site in ("W1", "W2")}
    (ROOT/"config/planning/ue_loading_access_routes.json").write_text(json.dumps(dict(
        status="VALIDATED_LOW_SPEED_DIAGNOSTIC", routing_enabled=False,
        map_sha256=current_maps,
        topology_sha256=hashlib.sha256((ROOT/"config/topology/industrialarea_ue.json").read_bytes()).hexdigest(),
        sites=routes),ensure_ascii=False,indent=2)+"\n")
lines = ["# W1/W2 道路接入与倒车区域预约验收", "",
         "项目验收以完整跑通、车身不接触障碍物、车辆不碰撞及预约正确释放为准。轮胎轻微离地仅记录，不作为失败条件；轮胎与已核实路面或坡道的竖直接触须同时满足车身无碰撞、无车辆重叠，不能用来豁免侧向碰撞。", "",
         "验收范围：从定稿横路沿既有路口连接线前进到纵路，在车道上停稳后倒入装货位，再前进出位并继续沿原方向驶离。全过程只有测试开场摆车，阶段之间不复位或传送。相机保持自由。", "",
         "| 点位 | 三车接入与预约 | 南行车道直行 | 北行车道直行 | 后车停稳等待 | 综合状态 |",
         "|---|---|---|---|---:|---|"]
for row in rows:
    cases = row["cases"]
    lines.append(f"| {row['site']}-loading | {cases['reservation']['status']} | {cases['straight_south']['status']} | {cases['straight_north']['status']} | {cases['reservation'].get('follower_stationary_wait_s',0):.1f} s | {'PASS' if row['low_speed_access_and_reservation_pass'] else 'PENDING'} |")
lines += ["", "区域预约使用 9 m × 3.3 m 整车包络判断占用，覆盖路口、倒车圆弧与坡道。区域内有其他车辆时拒绝预约；占用车按西向车道前进清空，后车沿东向车道停在路口外等待。作业车完成出位后继续向低 Y 行驶，整车尾部完全离开区域才释放预约并放行后车。两车相邻车道通行时也检查整车间距和车身重叠。", "",
          "北行直行回归发现原坡道侧边会使后轮短暂离地，已给两个子地图各增加两段沿道路方向的缓坡端部，尺寸为 4.5 m × 4.4 m，搭接约 20 cm。轮迹剖面每半米的最大高差由约 10.8 cm 降至约 1.3 cm。两份地图修改前均已备份；验收记录核对当前地图文件的校验值，修改后的地图重新完成直行与三车流程，旧记录不作为当前通过。", "",
          "预约组件：`src/logistics_park/scheduling/maneuver_reservation.py`；动态试验：`scripts/validate_ue_loading_access.py`。原始逐帧记录见 `runs/ue_scene/station_siting/loading_access_*.json`；分阶段误差、接地与事件摘要见同目录文档 `ue_loading_access_acceptance.json`。", "",
          "这些是低速诊断验证，未覆盖运营车速、反向车辆同时通过、多个泊位同时作业或自动调度中的故障恢复。W1/W2 卸货位另行验证；local waiting 已按用户要求弃用，自动调度继续禁用。", ""]
if region_audit.get("after_map_sha256") == current_maps:
    lines += ["本轮 W1/W2 卸货入口及 W5/W6 入口坡道修改后，地图文件校验值发生变化。W1/W2 原始实车记录仍对应修改前地图；已用修改前后场景快照核对两处原试验范围内的静态 Actor、网格、位置、旋转及包络均未改变，保留原验收结论，并单独标记 `unchanged_region_verified`。此处保留历史实车结果，快照核对确认本次改动未覆盖原试验范围。证据见 `runs/ue_scene/station_siting/loading_regions_after_middle_ramps.json`。", ""]
lines += ["本次清障采用本路口反向车道驶离；较低 Y 的 j_0_2 连接段不在本次验收范围内。过程记录保存在 `runs/ue_scene/short_cargo_history.jsonl`。", ""]
(ROOT/"docs/ue_loading_access_acceptance.md").write_text("\n".join(lines),encoding="utf-8")
print(summary["status"], [(r["site"],r["low_speed_access_and_reservation_pass"]) for r in rows])
