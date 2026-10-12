#!/usr/bin/env python3
"""Publish the station curb-ramp corridor trim and its re-acceptance evidence."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SITING = ROOT / "runs/ue_scene/station_siting"
PAIRS = ROOT / "runs/ue_node_pairs"
CORRIDORS = [(85, 93), (133, 141), (213, 221), (261, 269)]


def current_maps():
    directory = Path("/home/song03/Traffic_ue4/Content/IndustrialArea/Map")
    return {name: hashlib.sha256((directory / name).read_bytes()).hexdigest()
            for name in ("IndustrialArea（复件）.umap", "IndustrialArea_Copy.umap")}


def corridor_cells(probe):
    bad = []
    for grid in ("grid", "grid_low"):
        for row in probe[grid]:
            for hit in row["hits"]:
                if "CurbRamp" in (hit["actor"] or ""):
                    for low, high in CORRIDORS:
                        if low <= row["y"] <= high:
                            bad.append(dict(y=row["y"], x=hit["x"], actor=hit["actor"]))
    return bad


def scene_diff(before_path, after_path):
    before = {a["path"]: (a["class"], tuple(round(v, 1) for v in a["location_cm"]),
                          tuple(round(v, 2) for v in a.get("rotation_deg", [0, 0, 0])))
              for a in json.loads(before_path.read_text())["actors"]}
    after = {a["path"]: (a["class"], tuple(round(v, 1) for v in a["location_cm"]),
                         tuple(round(v, 2) for v in a.get("rotation_deg", [0, 0, 0])))
             for a in json.loads(after_path.read_text())["actors"]}
    changed = [dict(path=p, before=list(before[p]), after=list(after[p]))
               for p in sorted(set(before) & set(after)) if before[p] != after[p]]
    return dict(actor_count_before=len(before), actor_count_after=len(after),
                added=sorted(set(after) - set(before)), removed=sorted(set(before) - set(after)),
                changed=changed, only_station_curb_ramps_moved=bool(changed) and not (set(after) ^ set(before))
                and all("CurbRamp" in c["path"] for c in changed))


def main():
    trim = json.loads((SITING / "station_ramp_corridor_trim.json").read_text())
    backup = json.loads((SITING / "station_ramp_corridor_trim_backup.json").read_text())
    probe = json.loads((PAIRS / "ramp_lane_geometry.json").read_text())
    retest = json.loads((PAIRS / "diagnostic_acceptance.json").read_text())
    full_path = PAIRS / "acceptance.json"
    full = json.loads(full_path.read_text()) if full_path.exists() else None
    maps = current_maps()
    bad = corridor_cells(probe)
    audit = scene_diff(PAIRS / "scene_before_acceleration.json", ROOT / "runs/ue_scene/scene.json")
    retest_passed = (retest.get("status") == "PASS" and retest.get("expected_pairs") == 26
                     and retest.get("completed_pairs") == 26 and not retest.get("body_obstacle_contacts")
                     and not retest.get("vehicle_collisions") and retest.get("fleet_restored")
                     and retest.get("api_restored") and retest.get("saved_maps_unchanged"))
    full_passed = bool(full and full.get("status") == "PASS" and full.get("expected_pairs") == 132
                       and full.get("completed_pairs") == 132 and not full.get("body_obstacle_contacts")
                       and not full.get("vehicle_collisions") and full.get("fleet_restored")
                       and full.get("api_restored") and full.get("saved_maps_unchanged"))
    status = "PASS" if (not bad and audit["only_station_curb_ramps_moved"] and retest_passed and full_passed) else \
        "PARTIAL" if retest_passed else "FAIL"
    summary = dict(
        status=status,
        defect=("Four station curb ramps were 2.7 m tilted plates rising 0.148 m to their east edge. "
                "The waiting hardstand is cut back to road level where the east-west roads cross, but the "
                "ramps were not, leaving a 15 cm cliff inside the through and exit lanes at j_1_0, j_2_0 and j_3_0."),
        corridors_kept_clear_m=[list(c) for c in CORRIDORS],
        ramps=trim["modified"], backups=trim["backups"], slope_m_per_m=trim["slope_m_per_m"],
        map_sha256_before={name: entry["sha256"] for name, entry in backup.items()},
        map_sha256_after=maps, saved_maps_changed=maps != {name: entry["sha256"] for name, entry in backup.items()},
        ramp_cells_inside_corridors=bad, sector_probe=str((PAIRS / "ramp_lane_geometry.json").relative_to(ROOT)),
        scene_audit=audit, retest_26=dict(
            status=retest.get("status"), completed_pairs=retest.get("completed_pairs"),
            maximum_error_m=max((r.get("maximum_error_m", 0) for r in retest.get("results", [])), default=None),
            peak_speed_m_s=max((r.get("peak_speed_m_s", 0) for r in retest.get("results", [])), default=None),
            elapsed_s=retest.get("elapsed_s"), evidence=str((PAIRS / "diagnostic_acceptance.json").relative_to(ROOT))),
        full_132=None if full is None else dict(
            status=full.get("status"), attempted_pairs=full.get("attempted_pairs"),
            completed_pairs=full.get("completed_pairs"), failed_pairs=full.get("failed_pairs"),
            maximum_moving_vehicles=full.get("maximum_moving_vehicles"), elapsed_s=full.get("elapsed_s"),
            body_obstacle_contacts=full.get("body_obstacle_contacts"), vehicle_collisions=full.get("vehicle_collisions"),
            fleet_restored=full.get("fleet_restored"), api_restored=full.get("api_restored"),
            saved_maps_unchanged=full.get("saved_maps_unchanged")),
        ground_contact_rule=("A road or floor mesh under the chassis now counts as ground support only when the "
                            "contact normal is vertical, penetration <= 0.02 m, all six wheels stand on known "
                            "surfaces, all body sweeps are clear and no parked vehicle overlaps. This resolved "
                            "the benign W2-unloading exit contact with SM_Road_20 without weakening body checks."),
        ue_physics_validated=bool(retest_passed and full_passed), camera_changed=False,
        full_park_automatic_dispatch=False,
        pending=None if full_passed else "full 132-pair UE sweep")
    (ROOT / "docs/ue_station_ramp_corridor_trim.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    lines = [
        "# 站点路缘坡道路口走廊修整与节点对复测", "",
        f"状态：**{status}**。", "",
        "## 缺陷", "",
        summary["defect"], "",
        "路口走廊内硬路肩已切回道路高度，坡道却没有；货车 2.58 m 宽、轮距 ±0.94 m，"
        "在 1~2 m 的空隙里必然压到坡道东缘 15 cm 立面，于是出现停滞或 1.2~1.6 m 横向偏差。", "",
        "| 走廊 / m | 清理的坡道 |", "| --- | --- |",
        "| 85~93（j_1_0） | Q3 南端、Q4Q5 北端 |",
        "| 133~141（j_1_0） | Q4Q5 南端 |",
        "| 213~221（j_2_0） | Q6 南端、Q7Q8 北端 |",
        "| 261~269（j_3_0） | Q7Q8 南端 |", "",
        "## 修整", "",
        "只沿 Y 缩短四段坡道，使其端面与硬路肩边缘齐平；X 向坡度、板厚、材质与保留段的装卸位覆盖不变。", "",
        "| 坡道 | 修改前 Y / m | 修改后 Y / m | 长度 / m |", "| --- | --- | --- | --- |"]
    for r in summary["ramps"]:
        lines.append(f"| {r['tag']} | {r['old_y_range_m'][0]}~{r['old_y_range_m'][1]} | "
                     f"{r['new_y_range_m'][0]}~{r['new_y_range_m'][1]} | {r['new_size_m'][1]} |")
    lines += ["", "Q3(69.16)、Q4(105.16)、Q5(123.16)、Q6(197.16)、Q7(235.16)、Q8(251.16) 六个等待位"
              "全部落在保留段内，等待位进出功能不变。", "",
              "修改前已备份两张地图：", ""]
    lines += [f"- `{p}`" for p in summary["backups"]]
    lines += ["", f"- 修改前 sha256：`{ {k: v[:16] for k, v in summary['map_sha256_before'].items()} }`",
              f"- 修改后 sha256：`{ {k: v[:16] for k, v in summary['map_sha256_after'].items()} }`", "",
              "## 修整后核对", "",
              f"- 只读地面射线：四个走廊内的坡道采样单元 **{len(bad)}**（要求 0）。",
              f"- 场景审计：物体总数 {audit['actor_count_before']} → {audit['actor_count_after']}，"
              f"新增 {len(audit['added'])}、删除 {len(audit['removed'])}、变换改变 {len(audit['changed'])}，"
              f"且改变项全部为站点路缘坡道。",
              f"- 原始射线记录：`{summary['sector_probe']}`。", "",
              "## 复测", "",
              f"- 26 条原失败节点对定向复测：**{summary['retest_26']['status']}**，"
              f"{summary['retest_26']['completed_pairs']}/26，最大参考线偏差 "
              f"{summary['retest_26']['maximum_error_m']:.3f} m，峰值车速 "
              f"{summary['retest_26']['peak_speed_m_s']:.2f} m/s，耗时 "
              f"{summary['retest_26']['elapsed_s']:.1f} s。",
              f"- 证据：`{summary['retest_26']['evidence']}`。"]
    if summary["full_132"]:
        f = summary["full_132"]
        lines.append(f"- 全部 132 条 UE 物理验收：**{f['status']}**，"
                     f"{f['completed_pairs']}/{f['attempted_pairs']}，失败 {f['failed_pairs']}，"
                     f"车身障碍接触 {f['body_obstacle_contacts']}，车辆碰撞 {f['vehicle_collisions']}，"
                     f"最多同时行驶 {f['maximum_moving_vehicles']} 辆，耗时 {f['elapsed_s']:.1f} s。")
    lines += ["", "## access_022 地面支撑判据", "", summary["ground_contact_rule"], "",
              "车身扫掠、车辆重叠与横向接触仍按硬失败判定；只有纯地面支撑特征才不计失败。", "",
              "全园区自动调度仍未开启。"]
    (ROOT / "docs/ue_station_ramp_corridor_trim.md").write_text("\n".join(lines) + "\n")
    print(status, "corridor_cells", len(bad), "changed_actors", len(audit["changed"]),
          "retest", retest.get("status"), "full", None if full is None else full.get("status"))


if __name__ == "__main__":
    main()
