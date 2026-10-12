#!/usr/bin/env python3
"""Summarize approved station evidence without enabling unverified routes."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "config/planning/ue_station_proposal.json"
ACCESS = ROOT / "runs/ue_scene/station_siting/access_corridors.json"
DYNAMIC = ROOT / "runs/ue_scene/station_siting/roadside_dynamic.json"
SITE_TRIALS = {
    "Q3": ("Q3", "Q3-exit-gentle", "Q3-main-lane"),
    "Q4": ("Q4-gentle", "Q4-exit-gentle", "Q4-main-lane"),
    "Q5": ("Q5", "Q5-exit-gentle", "Q5-main-lane"),
}
OUT = ROOT / "config/planning/ue_station_validation.json"
DOC = ROOT / "docs/ue_station_validation.md"

plan = json.loads(PLAN.read_text())
access = {p["id"]: p for p in json.loads(ACCESS.read_text())["points"]}
dynamic = {c["id"]: c for c in json.loads(DYNAMIC.read_text())["cases"]}
def passed(trial):
    path = ROOT / f"runs/ue_scene/station_siting/dynamic_entry_{trial}.json"
    if not path.exists():
        return False
    record = json.loads(path.read_text())
    return record["status"] == "PASS" and record.get("vehicle_restored", False)


site_access = {site: all(passed(trial) for trial in trials)
               for site, trials in SITE_TRIALS.items()}
reverse_access = {}
loading_access = {}
service_manifest = ROOT/"docs/ue_remaining_service_acceptance.json"
service_access = {r["id"]:r for r in json.loads(service_manifest.read_text())["sites"]} if service_manifest.exists() else {}
waiting_manifest=ROOT/"docs/ue_waiting_access_acceptance.json"
waiting_access={r["id"]:r for r in json.loads(waiting_manifest.read_text())["sites"]} if waiting_manifest.exists() else {}
loading_manifest = ROOT/"docs/ue_loading_access_acceptance.json"
if loading_manifest.exists():
    loading_access = {s["site"]: s for s in json.loads(loading_manifest.read_text())["sites"]}
for site in ("W1", "W2"):
    path = ROOT / f"runs/ue_scene/station_siting/dynamic_entry_{site}-loading.json"
    if path.exists():
        record = json.loads(path.read_text())
        reverse_access[site] = (record.get("status") == "PASS"
                                and record.get("maneuver") == "reverse"
                                and record.get("follow", {}).get("status") == "PASS"
                                and record.get("vehicle_restored", False))
rows = []
for station in plan["stations"]:
    for kind in ("loading", "unloading", "waiting"):
        berth = station[kind]
        if not berth:
            continue
        label = station["id"]+"-"+kind
        corridor = access[label]
        stop_index = 0 if kind == "loading" else 1
        case = dynamic.get(station["id"])
        driven = bool(case and case["status"] == "PASS" and kind != "waiting"
                      and len(case["stops"]) > stop_index)
        rows.append({
            "id": label, "warehouse": station["id"], "kind": kind,
            "static_parking_pass": berth["static_parking_probe"] == "PASS",
            "straight_back_m": corridor["backward_clear_m"],
            "straight_front_m": corridor["forward_clear_m"],
            "straight_10m_both_sides": min(corridor["backward_clear_m"],
                                           corridor["forward_clear_m"]) >= 10,
            "single_truck_stop_go_pass": driven,
            "single_truck_entry_exit_pass": ((kind == "loading"
                                             and reverse_access.get(station["id"], False))
                                            or service_access.get(label,{}).get("continuous_entry_exit_pass",False)),
            "normal_lane_access_and_reservation_pass": (
                kind == "loading" and loading_access.get(station["id"], {}).get(
                    "low_speed_access_and_reservation_pass",False)),
            "traffic_and_access_validated": False,
            "routing_enabled": False,
        })
for berth in plan["shared_waiting"]:
    label = berth["id"]
    corridor = access[label]
    rows.append({
        "id": label, "kind": "waiting", "static_parking_pass": True,
        "straight_back_m": corridor["backward_clear_m"],
        "straight_front_m": corridor["forward_clear_m"],
        "straight_10m_both_sides": min(corridor["backward_clear_m"],
                                       corridor["forward_clear_m"]) >= 10,
        "single_truck_stop_go_pass": False,
        "single_truck_entry_exit_pass": (site_access.get(label, False) or
                                         waiting_access.get(label,{}).get("continuous_entry_exit_pass",False)),
        "normal_lane_access_and_reservation_pass": False,
        "traffic_and_access_validated": False,
        "routing_enabled": False,
    })
summary = {
    "layout_approved": True, "scope": "UE station access validation; no automatic station routing yet",
    "station_count": len(plan["stations"]), "berth_count": len(rows),
    "static_parking_pass_count": sum(r["static_parking_pass"] for r in rows),
    "straight_10m_both_sides_count": sum(r["straight_10m_both_sides"] for r in rows),
    "single_truck_stop_go_pass_count": sum(r["single_truck_stop_go_pass"] for r in rows),
    "single_truck_entry_exit_pass_count": sum(r["single_truck_entry_exit_pass"] for r in rows),
    "waiting_entry_exit_pass_count":sum(r["single_truck_entry_exit_pass"] for r in rows if r["kind"]=="waiting"),
    "normal_lane_access_and_reservation_pass_count": sum(
        r["normal_lane_access_and_reservation_pass"] for r in rows),
    "routing_enabled_count": 0, "points": rows,
    "evidence": [
        "runs/ue_scene/station_siting/access_corridors.json",
        "runs/ue_scene/station_siting/roadside_dynamic_W3.json",
        "runs/ue_scene/station_siting/roadside_dynamic_W4.json",
        *[f"runs/ue_scene/station_siting/dynamic_entry_{trial}.json"
          for trials in SITE_TRIALS.values() for trial in trials],
        *[f"runs/ue_scene/station_siting/dynamic_entry_{site}-loading.json"
          for site in ("W1", "W2") if reverse_access.get(site)],
        *(["docs/ue_loading_access_acceptance.json"] if loading_access else []),
        *(["docs/ue_remaining_service_acceptance.json"] if service_access else []),
        *(["docs/ue_waiting_access_acceptance.json"] if waiting_access else []),
    ],
}
OUT.write_text(json.dumps(summary, ensure_ascii=False, indent=2)+"\n")
short = [r["id"] for r in rows if not r["straight_10m_both_sides"]]
doc = f"""# UE 仓库点位验证状态

六仓库、{summary['berth_count']} 个点位的布局已审批，并在 UE PIE 中画出泊位边框。**当前没有自动调度启用的仓库点位。**

| 验证层级 | 数量 | 含义 |
|---|---:|---|
| 静态可停车 | {summary['static_parking_pass_count']}/{summary['berth_count']} | 地面与 9 m × 3.3 m 包络无静态碰撞 |
| 点位沿朝向前后各 10 m 直线通道 | {summary['straight_10m_both_sides_count']}/{summary['berth_count']} | 仅直线走廊初筛，不能替代道路接入路径 |
| 单车停靠、起步、驶离实测 | {summary['single_truck_stop_go_pass_count']}/{summary['berth_count']} | W3/W4 的装货、卸货四点通过 |
| 单车从车道驶入、停稳并驶出 | {summary['single_truck_entry_exit_pass_count']}/{summary['berth_count']} | 已完成连续道路进出的装卸位及共享等待位 |
| 共享等待位连续进出 | {summary['waiting_entry_exit_pass_count']}/12 | 单车驶入、停稳等待、离位后返回合法车道 |
| 正常道路接入、区域预约及相邻车道回归 | {summary['normal_lane_access_and_reservation_pass_count']}/{summary['berth_count']} | 完整道路进出、三车清障/等待/释放、坡道旁双向直行 |
| 可接入自动调度 | 0/{summary['berth_count']} | 尚缺建筑出入口、等待位进出及多车交通冲突验收 |

直线走廊不足的点位：{', '.join(short)}。这不直接证明无法使用；有些位置可能需要转弯、倒车、围栏开口或调整路缘。当前保留已审批坐标，在 UE 中逐点验证或作最小调整，并在确认完整路径后才打开调度。

W3/W4 的四个装卸点在当前短货厢卡车单车测试中完成驶入、停稳、再驶离；测试没有让其他车辆同时通行，尚不能把这四点视为多车运营可用。宽路北行右车道及端部调头区仍需预约/冲突控制。

Q3/Q4/Q5 已在 UE 地图中清除经整车碰撞试验确认的局部矮墙、杆件、标志、箱体和低矮装饰叶片，并沿宽路东侧路缘增加两段坡道。当前短货厢卡车从南行车道驶入三处等待位，停稳后以缓转 S 形路线驶回南行右车道，六段低速实测均无碰撞；三次坡道旁的南行右车道直行回归也通过。三处进出路线最大路径偏差均不超过 0.24 m，最大车轮接地偏差约 9.6 cm。较急的驶离路线曾碰到路缘，已弃用。地图改动、逐次备份及原始轨迹见 `runs/ue_scene/station_siting/`；仍需多车会车、泊位预约、速度扩展和可视检查，三处暂未启用调度。

W1/W2 装货位通过短货厢卡车**倒车入位、停稳、前进出位**的连续单车低速实测。原有路缘约 15–18 cm 高差，已在两个子地图各加两段经轮迹核对的局部坡道；W1 先行调试、W2 按最终尺寸独立复测。两处试验车身扫描无障碍碰撞，车轮接地偏差及路线误差见原始记录。后续正常道路接入、三车区域预约和坡道旁双向车道直行的独立验收见[道路接入与预约验收](ue_loading_access_acceptance.md)。完成该层验收的点位数为 {summary['normal_lane_access_and_reservation_pass_count']}；运营速度、多个泊位同时作业和调度故障恢复仍待验证，暂不启用调度。

剩余 Q1/Q2、Q6～Q12 的进出、坡道旁双向车道直行及 Q1/Q2 停放时 W5 作业通道的逐项结果见[等待位进出验收](ue_waiting_access_acceptance.md)。Q6/Q8 本轮进一步清理四段小围挡，Q9～Q12 补设共享路缘坡道；每个点位以独立实车状态为准。等待位连续进出累计通过 {summary['waiting_entry_exit_pass_count']}/12，仍需接入共享通道预约及运输任务。

W1/W2 的 local waiting 已按用户要求弃用，释放卸货通道；当前共 12 个装卸位、12 个共享等待位。W1/W2 卸货位、W5/W6 装卸位的逐点结果见[剩余装卸位进出验收](ue_remaining_service_acceptance.md)。轮胎轻微离地只记录，是否通过取决于完整跑通、车身无障碍接触及无车辆碰撞。

机器可读状态：`config/planning/ue_station_validation.json`；原始证据：`runs/ue_scene/station_siting/`。UE 仿真标记没有保存到地图，也没有改变相机控制。
"""
DOC.write_text(doc, encoding="utf-8")
print(summary["static_parking_pass_count"], "static;",
      summary["straight_10m_both_sides_count"], "straight corridors;",
      summary["single_truck_stop_go_pass_count"], "single-truck stops")
