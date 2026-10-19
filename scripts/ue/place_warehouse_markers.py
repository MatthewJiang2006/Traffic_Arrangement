"""Show approved warehouse service/waiting envelopes in the active PIE world."""
import json
import math
from pathlib import Path

import unreal

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "config/planning/ue_station_proposal.json"
OUTPUT = ROOT / "runs/ue_scene/station_siting/marker_placement.json"
plan = json.loads(CONFIG.read_text(encoding="utf-8"))
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
unreal.SystemLibrary.flush_persistent_debug_lines(world)
unreal.SystemLibrary.flush_debug_strings(world)
colors = {
    "loading": unreal.LinearColor(1, .15, .1, 1),
    "unloading": unreal.LinearColor(.55, .15, 1, 1),
    "waiting": unreal.LinearColor(.1, .95, .25, 1),
}
rows = []
points = [(s["id"] + "-" + role, role, s[role])
          for s in plan["stations"] for role in ("loading", "unloading", "waiting")
          if s[role] is not None]
points += [(p["id"], "waiting", p) for p in plan["shared_waiting"]]
for label, role, point in points:
    x, y = point["xy_m"]
    z = point["ground_z_m"]
    center = unreal.Vector(x*100-origin.x, y*100-origin.y, (z+.06)*100-origin.z)
    yaw = point["heading_deg"]
    angle = math.radians(yaw)
    long_dir = unreal.Vector(450*math.cos(angle), 450*math.sin(angle), 0)
    cross_dir = unreal.Vector(-165*math.sin(angle), 165*math.cos(angle), 0)
    corners = [center-long_dir-cross_dir, center+long_dir-cross_dir,
               center+long_dir+cross_dir, center-long_dir+cross_dir]
    color = colors[role]
    for a, b in zip(corners, corners[1:]+corners[:1]):
        unreal.SystemLibrary.draw_debug_line(world, a, b, color, 86400, 8)
    point_above = center + unreal.Vector(0, 0, 110)
    unreal.SystemLibrary.draw_debug_string(world, point_above, label, None, color, 86400)
    rows.append({"id": label, "role": role, "xy_m": [x, y], "heading_deg": yaw,
                 "ground_z_m": z, "parking_envelope_m": point["parking_envelope_m"]})
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_text(json.dumps({"status": "PASS", "count": len(rows), "markers": rows,
                              "map_saved": False, "camera_changed": False}, indent=2) + "\n")
