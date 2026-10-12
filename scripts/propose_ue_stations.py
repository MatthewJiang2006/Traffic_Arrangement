#!/usr/bin/env python3
"""Generate a reviewable six-warehouse plan from read-only UE clearance probes."""
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch, Rectangle

ROOT = Path(__file__).resolve().parents[1]
SURVEY = ROOT / "runs/ue_scene/station_siting"
FILES = [
    "ground_clearance.json", "expanded_ground_clearance.json",
    "road_service_ground_clearance.json", "waiting_ground_clearance.json",
    "waiting_directional_clearance.json",
]
SCENE = json.loads((ROOT / "runs/ue_scene/scene.json").read_text())
ROADS = json.loads((ROOT / "config/topology/industrialarea_ue.json").read_text())
evidence = {}
for file in FILES:
    for row in json.loads((SURVEY / file).read_text())["rows"]:
        if row["clear"]:
            evidence[(*row["xy_m"], row["heading_deg"])] = (row, file)


def berth(x, y, heading, kind):
    row, source = evidence[(x, y, heading)]
    return {
        "xy_m": [x, y], "heading_deg": heading, "ground_z_m": row["ground_z_m"],
        "surface_actor": row["surface_actor"], "parking_envelope_m": [9.0, 3.3],
        "static_parking_probe": "PASS", "probe_source": f"runs/ue_scene/station_siting/{source}",
        "kind": kind,
    }


def station(id, description, actor, building, load, unload, wait=None):
    return {
        "id": id, "description": description, "building_actor": actor, "building_bounds_m": building,
        "loading": berth(*load, "roadside service" if id in ("W3", "W4") else "off-road hardstand"),
        "unloading": berth(*unload, "roadside service" if id in ("W3", "W4") else "off-road hardstand"),
        "waiting": berth(*wait, "off-road hardstand") if wait else None,
    }


stations = [
    station("W1", "保留：低 Y 园区右侧低棚厂房", "existing east-side low-roof building",
            [22.8, 97.2, 36.3, 122.2], (28, 124, 0), (20, 112, 90)),
    station("W2", "保留：高 Y 园区右侧低棚厂房", "existing east-side low-roof building",
            [22.8, 225.2, 36.3, 250.2], (28, 252, 0), (20, 240, 90)),
    station("W3", "新增：宽路低 Y 端西侧建筑", "SM_Build_54 / SM_Floor_5 / SM_Floor_6",
            [-119.2, 49.7, -104.4, 91.2], (-98, 66, 90), (-98, 80, 90)),
    station("W4", "新增：宽路高 Y 端西侧建筑", "SM_Build_642",
            [-128.3, 280.3, -103.7, 305.3], (-98, 274, 90), (-98, 288, 90)),
    station("W5", "新增：中间建筑排低 Y 侧", "SM_Build_197",
            [-65.1, 143.4, -37.7, 169.1], (-40, 173, 0), (-28, 173, 0)),
    station("W6", "新增：中间建筑排高 Y 侧", "SM_Build_169",
            [-33.1, 195.0, -17.8, 209.3], (-36, 184, 0), (-24, 184, 0)),
]
shared = [
    {"id": "Q1", **berth(-64, 173, 0, "off-road hardstand")},
    {"id": "Q2", **berth(-52, 173, 0, "off-road hardstand")},
    {"id": "Q3", "zone": "W3 low-end curb", **berth(-82, 68, 270, "road-edge shoulder")},
    {"id": "Q4", "zone": "W3 lower road curb", **berth(-82, 104, 270, "road-edge shoulder")},
    {"id": "Q5", "zone": "W3 lower road curb", **berth(-82, 122, 270, "road-edge shoulder")},
    {"id": "Q6", "zone": "central connector curb", **berth(-82, 196, 270, "road-edge shoulder")},
    {"id": "Q7", "zone": "W4 upper road curb", **berth(-82, 234, 270, "road-edge shoulder")},
    {"id": "Q8", "zone": "W4 upper road curb", **berth(-82, 250, 270, "road-edge shoulder")},
    {"id": "Q9", "zone": "W1 inner roadside hardstand", **berth(-20, 110, 90, "off-road hardstand")},
    {"id": "Q10", "zone": "W1 inner roadside hardstand", **berth(-20, 124, 90, "off-road hardstand")},
    {"id": "Q11", "zone": "W2 inner roadside hardstand", **berth(-20, 238, 90, "off-road hardstand")},
    {"id": "Q12", "zone": "W2 inner roadside hardstand", **berth(-20, 252, 90, "off-road hardstand")},
]


def bounds(p):
    x, y = p["xy_m"]
    w, h = (9.0, 3.3) if p["heading_deg"] in (0, 180) else (3.3, 9.0)
    return x-w/2, y-h/2, x+w/2, y+h/2


points = [(s["id"]+"-"+kind, s[kind]) for s in stations
          for kind in ("loading", "unloading", "waiting") if s[kind]]
points += [(p["id"], p) for p in shared]
for i, (name_a, a) in enumerate(points):
    ax0, ay0, ax1, ay1 = bounds(a)
    for name_b, b in points[i+1:]:
        bx0, by0, bx1, by1 = bounds(b)
        if ax0 < bx1 and bx0 < ax1 and ay0 < by1 and by0 < ay1:
            raise ValueError(f"parking envelopes overlap: {name_a} / {name_b}")

proposal = {
    "status": "APPROVED_FOR_UE_VALIDATION",
    "user_approved": True,
    "coordinate_frame": "UE absolute world metres; parking-envelope centres, not Actor spawn positions",
    "stations": stations, "shared_waiting": shared,
    "loading_capacity_per_station": 1, "unloading_capacity_per_station": 1,
    "shared_waiting_capacity": len(shared), "total_waiting_capacity": len(shared),
    "retired_waiting": [
        {"id": "W1-local", "xy_m": [20,102], "reason": "用户弃用：阻挡卸货位通道"},
        {"id": "W2-local", "xy_m": [20,230], "reason": "用户弃用：阻挡卸货位通道"}],
    "waiting_assignment": {
        "W1": ["Q9", "Q10"], "W2": ["Q11", "Q12"],
        "W3": ["Q3", "Q4", "Q5"], "W4": ["Q7", "Q8"],
        "W5": ["Q1", "Q2", "Q6"], "W6": ["Q1", "Q2", "Q6"],
    },
    "scene_modified": False, "entry_exit_driving_validated": False,
    "road_service_rule": "W3/W4 occupy the northbound right lane while serving; reserve the lane and terminal turn zone. Adjacent-lane passing requires a separate swept-clearance check before authorization.",
    "required_after_approval": [
        "confirm curb/fence access and service-side reach from each building",
        "validate complete truck approach, stop, loading/unloading and departure in UE",
        "verify shoulder waiting entry/exit, tire contact and live-lane clearance; then enforce lane/berth reservations and queue limits",
        "add warehouse/berth/queue nodes to transport topology and task state machine",
    ],
    "survey_sources": [f"runs/ue_scene/station_siting/{f}" for f in FILES],
}
(ROOT/"config/planning").mkdir(exist_ok=True)
(ROOT/"config/planning/ue_station_proposal.json").write_text(
    json.dumps(proposal, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")

fig, ax = plt.subplots(figsize=(11, 14))
for actor in SCENE["actors"]:
    if not actor["meshes"] or actor["class"] == "CameraActor":
        continue
    x, y, z = (v/100 for v in actor["bounds_origin_cm"])
    hx, hy, hz = (v/100 for v in actor["bounds_extent_cm"])
    if hx <= 0 or hy <= 0 or hx > 100 or hy > 100 or z+hz < 1.45 or z-hz > 6:
        continue
    if any("SM_Road_" in m for m in actor["meshes"]):
        continue
    ax.add_patch(Rectangle((x-hx, y-hy), 2*hx, 2*hy, color="#64748b", alpha=.22, lw=0))
for edge in ROADS["edges"]:
    if edge["edge_id"].startswith("road_"):
        xs, ys = zip(*edge["centerline"])
        ax.plot(xs, ys, color="#dbe4ed", lw=4, zorder=0)
        ax.plot(xs, ys, color="#64748b", lw=.5, zorder=1)


def mark(p, label, color, offset=(8, 8)):
    x, y = p["xy_m"]
    x0, y0, x1, y1 = bounds(p)
    ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, facecolor=color,
                           edgecolor="white", lw=1, zorder=5))
    ax.annotate(label, (x, y), xytext=offset, textcoords="offset points",
                fontsize=8, weight="bold", color=color,
                bbox=dict(facecolor="white", edgecolor=color, alpha=.93, pad=1.5), zorder=6)


for s in stations:
    x0, y0, x1, y1 = s["building_bounds_m"]
    ax.add_patch(Rectangle((x0, y0), x1-x0, y1-y0, facecolor="#dbeafe",
                           edgecolor="#2563eb", alpha=.65, zorder=2))
    ax.text((x0+x1)/2, (y0+y1)/2, s["id"], ha="center", va="center",
            fontsize=12, color="#1d4ed8", weight="bold", zorder=4)
    for kind, suffix, color in (("loading", "L", "#dc2626"),
                                ("unloading", "U", "#7c3aed"),
                                ("waiting", "Q", "#d97706")):
        if s[kind]:
            mark(s[kind], s["id"]+"-"+suffix, color)
for p in shared:
    mark(p, p["id"], "#059669", offset=(-3, -17) if p["id"] in ("Q1", "Q2") else (5, 2))
ax.set_xlim(-137, 66)
ax.set_ylim(45, 310)
ax.set_aspect("equal")
ax.grid(alpha=.18)
ax.set_xlabel("UE absolute X (m)")
ax.set_ylabel("UE absolute Y (m)")
ax.set_title("Approved six-warehouse layout — UE validation pending")
ax.legend(handles=[
    Patch(facecolor="#dbeafe", edgecolor="#2563eb", label="Warehouse building"),
    Patch(facecolor="#dc2626", label="Loading"),
    Patch(facecolor="#7c3aed", label="Unloading"),
    Patch(facecolor="#059669", label="Shared waiting"),
], loc="upper right", fontsize=8)
fig.tight_layout()
fig.savefig(ROOT/"docs/ue_station_proposal.svg")
fig.savefig(ROOT/"docs/ue_station_proposal.png", dpi=150)

rows = []
for s in stations:
    load, unload = s["loading"]["xy_m"], s["unloading"]["xy_m"]
    wait = s["waiting"]["xy_m"] if s["waiting"] else None
    rows.append(f"| {s['id']} | {s['description']} | {load[0]}, {load[1]} | "
                f"{unload[0]}, {unload[1]} | "
                + (f"{wait[0]}, {wait[1]}" if wait else "见等待位分区表") + " |")
doc = """# 六处仓库与装卸区（布局已审批，待 UE 动态验收）

保留原来的 W1、W2；在宽路低 Y 端、高 Y 端分别新增 W3、W4；在中间一排现有建筑中选 W5、W6。每处各有独立的装货位 L 和卸货位 U。W1、W2 的 local waiting 位因阻挡卸货通道，已按用户要求弃用；由 Q9/Q10、Q11/Q12 分别承接等待需求。原 Q1/Q2 保留，现有 Q1–Q12 共 **12 处共享等待位**。宽路两端使用现有路面，无需新建装卸坪。

![六仓库候选平面图](ue_station_proposal.png)

| 仓库 | 建筑与位置 | 装货 L 中心 X,Y (m) | 卸货 U 中心 X,Y (m) | 等待中心 X,Y (m) |
|---|---|---:|---:|---:|
""" + "\n".join(rows) + """

坐标是 UE 世界绝对米坐标、停车包络中心；车辆 Actor 生成点需按车体中心与挂厢偏移换算。图中上方是 +Y，不代表当前镜头中的“左/右”。每个静态包络为 9 m × 3.3 m，所有泊位互不重叠。

| 等待分区 | 泊位与中心 X,Y（m） | 建议服务仓库 |
|---|---|---|
| 中央共享 | Q1: -64,173；Q2: -52,173；Q6: -82,196 | W5、W6，满位时跨区调度 |
| 宽路低 Y 路缘 | Q3: -82,68；Q4: -82,104；Q5: -82,122 | W3 |
| 宽路高 Y 路缘 | Q7: -82,234；Q8: -82,250 | W4 |
| 中部道路旁硬化地 | Q9: -20,110；Q10: -20,124 | W1 |
| 中部道路旁硬化地 | Q11: -20,238；Q12: -20,252 | W2 |

Q3–Q8 是宽路**东侧路缘**等待位，车头沿 -Y 方向，贴近南行侧；Q9–Q12 在中部道路西侧空地，车头沿 +Y 方向。与现有 RoadGraph 的最近行驶车道边界比对，Q3–Q8 的静态包络尚有约 0.94 m 净距，Q9–Q12 至少约 4.34 m；这只是静态间距，尚需实车确认车门/货厢扫掠、轮胎接地、路缘进出和行驶车辆侧向净距。验证前它们只是候选容量，不能按已投运泊位调度。共享等待区按先到先服务预约，不能在主车道排队。

W3/W4 的 L、U 设在宽路北行**右车道**（X≈-98 m），贴近路西侧建筑；车辆沿 +Y 方向靠右驶入。装卸时必须预约车道、泊位及端部调头冲突区；相邻直行车道能否绕行，要等完整车身扫掠和侧向净距验证后决定，在此之前按双车道临时管制。路面停车不等于建筑到车厢的装卸通道已确认，实施前还需检查路缘、围栏与货物搬运路径。

W5 选 `SM_Build_197`（低 Y 一侧），W6 选 `SM_Build_169`（高 Y 一侧）；露天泊位在两栋建筑面向中间通道的现有硬化地面。W1/W2 的低棚净高不足以让当前挂厢驶入，车辆只在建筑外露天装卸。

当前 UE PIE 场景的只读地面射线与静态包络检测显示：12 个装卸位、12 个共享等待位均通过；原始记录见 `runs/ue_scene/station_siting/` 下五个 `*_clearance.json` 文件。这项检查不覆盖动态扫掠、货物搬运空间、入口/出口和道路交通冲突。审批后先实车验证，再将仓库拓扑、泊位预约、装卸时间与任务状态机接入正式仿真。

用户已批准该布局。坐标进入 UE 验证阶段；未经进出路径和整车动态验收的点位仍不可用于自动调度。机器可读配置：`config/planning/ue_station_proposal.json`。
"""
(ROOT/"docs/ue_station_proposal.md").write_text(doc, encoding="utf-8")
print(f"6 warehouses, 12 service berths, {len(shared)} waiting berths: proposal written")
