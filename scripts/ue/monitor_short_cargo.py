"""Audit the rigid truck's visible body envelope and tire contact in PIE."""
import builtins
import json
import math
import os
from pathlib import Path
import time
import unreal

ROOT = Path(__file__).resolve().parents[2]
REQUEST = getattr(builtins, "short_cargo_monitor_request", {})
HANDLE = REQUEST.get("handle", "short_cargo_monitor")
PREFIX = REQUEST.get("prefix", "short_cargo")
LIVE = ROOT/"runs/ue_scene"/(PREFIX+"_live.json")
HISTORY = ROOT/"runs/ue_scene"/(PREFIX+"_history.jsonl")
if getattr(builtins, HANDLE, None) is not None:
    unreal.unregister_slate_post_tick_callback(getattr(builtins, HANDLE))
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
truck = next(a for a in actors if a.get_name() == REQUEST.get("vehicle", "Car_0_0"))
visuals = [a for a in actors if a.get_class().get_name() == "BP_Truck_C"]
visual = next(a for a in visuals if a.get_attach_parent_actor() == truck)
vehicles = [a for a in actors if a.get_class().get_name() == "TrafficTruckPawn"]
origin = unreal.GameplayStatics.get_world_origin_location(world)
body_names = {"Truck_Body_00", "Truck_Body_01", "TrafficShortCargo"}
bodies = [c for c in visual.get_components_by_class(unreal.StaticMeshComponent) if c.get_name() in body_names]
wheels = [c for c in visual.get_components_by_class(unreal.StaticMeshComponent) if c.get_name().startswith("Wheel_")]
parked = [(a.get_attach_parent_actor().get_name(), c) for a in visuals if a != visual
          for c in a.get_components_by_class(unreal.StaticMeshComponent) if c.get_name() in body_names]
previous = {}
last = 0


def box(component):
    b = component.get_editor_property("static_mesh").get_bounds()
    centre = component.get_world_transform().transform_location(b.origin)
    scale = component.get_world_scale()
    half = unreal.Vector(b.box_extent.x*abs(scale.x), b.box_extent.y*abs(scale.y), b.box_extent.z*abs(scale.z))
    return centre, half, component.get_world_rotation()


def overlap(a, b):
    # Broad-phase vertical extent and SAT of horizontal body rectangles.
    pa, ea, ra = a
    pb, eb, rb = b
    if abs(pa.z-pb.z) >= ea.z+eb.z:
        return False
    angles = [math.radians(ra.yaw), math.radians(rb.yaw)]
    for t in (angles[0], angles[0]+math.pi/2, angles[1], angles[1]+math.pi/2):
        ax, ay = math.cos(t), math.sin(t)
        radius = sum(e.x*abs(math.cos(y)*ax+math.sin(y)*ay)
                     +e.y*abs(-math.sin(y)*ax+math.cos(y)*ay) for e, y in zip((ea, eb), angles))
        if abs((pa.x-pb.x)*ax+(pa.y-pb.y)*ay) >= radius:
            return False
    return True


def absolute(p):
    return [(p.x+origin.x)/100, (p.y+origin.y)/100, (p.z+origin.z)/100]


def publish(row):
    temp = LIVE.with_suffix(".tmp")
    temp.write_text(json.dumps(row))
    os.replace(temp, LIVE)
    with HISTORY.open("a") as stream:
        stream.write(json.dumps(row)+"\n")


def sample(delta):
    global last
    now = time.monotonic()
    if now-last < .1:
        return
    last = now
    try:
        row = {"monotonic_s": now, "game_time_s": unreal.GameplayStatics.get_time_seconds(world),
               "truck_ue_m": absolute(truck.get_actor_location()), "bodies": [], "wheels": [],
               "pitch_deg": truck.get_actor_rotation().pitch, "roll_deg": truck.get_actor_rotation().roll}
        if REQUEST.get('cache_frame_boxes',False):
            # All vehicle monitors in one UE frame can reuse body transforms.
            # Audits still trace each moving body and exclude its own truck.
            frame=row['game_time_s']
            cached=getattr(builtins,'fleet_body_frame_cache',None)
            if cached is None or cached[0]!=frame:
                all_boxes=[(a.get_attach_parent_actor().get_name(),box(c)) for a in visuals
                           for c in a.get_components_by_class(unreal.StaticMeshComponent) if c.get_name() in body_names]
                cached=(frame,all_boxes);builtins.fleet_body_frame_cache=cached
            candidates=[(name,b) for name,b in cached[1] if name!=truck.get_name()]
        else:
            candidates=[(name,box(c)) for name,c in parked]
        parked_boxes=[(name,b) for name,b in candidates
                      if not (REQUEST.get('exclude_inactive_fixture',False)
                              and name in (getattr(builtins,'node_pair_fleet_fixture',None) or {})
                              and not getattr(builtins,'node_pair_fleet_fixture')[name]['actor'].get_actor_enable_collision())]
        for c in bodies:
            centre, half, rotation = current = box(c)
            old, old_yaw = previous.get(c.get_name(), (centre, rotation.yaw))
            dyaw = math.radians(abs((rotation.yaw-old_yaw+180)%360-180))
            inflation = math.hypot(half.x, half.y)*math.sin(dyaw/2)
            swept_half = unreal.Vector(half.x+inflation, half.y+inflation, half.z)
            hit = unreal.SystemLibrary.box_trace_single(world, old, centre, swept_half, rotation,
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, vehicles+visuals, unreal.DrawDebugTrace.NONE, True)
            values = hit.to_tuple() if hit is not None else None
            hit_actor = values[9].get_name() if values and values[0] and values[9] else None
            other = next((name for name, candidate in parked_boxes if overlap(current, candidate)), None)
            row["bodies"].append({"name": c.get_name(), "centre_ue_m": absolute(centre),
                                  "half_size_m": [half.x/100, half.y/100, half.z/100],
                                  "yaw_deg": rotation.yaw, "static_sweep_hit": hit_actor,
                                  "parked_vehicle_overlap": other})
            previous[c.get_name()] = (centre, rotation.yaw)
        for c in wheels:
            p = c.get_world_location()
            b = c.get_editor_property("static_mesh").get_bounds()
            # Get the actual rotated tire's vertical support, including scale.
            tr = c.get_world_transform()
            bottom = min(tr.transform_location(unreal.Vector(b.origin.x+x*b.box_extent.x,
                         b.origin.y+y*b.box_extent.y, b.origin.z+z*b.box_extent.z)).z
                         for x in (-1, 1) for y in (-1, 1) for z in (-1, 1))
            hit = unreal.SystemLibrary.line_trace_single(world, p+unreal.Vector(0, 0, 300), p-unreal.Vector(0, 0, 1000),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, vehicles+visuals, unreal.DrawDebugTrace.NONE, True)
            values = hit.to_tuple() if hit is not None else None
            road = values[9].get_name() if values and values[0] and values[9] else None
            row["wheels"].append({"name": c.get_name(), "ue_m": absolute(p), "road_actor": road,
                                  "clearance_m": (bottom-values[5].z)/100 if values and values[0] else None})
        publish(row)
    except Exception as e:
        publish({"error": repr(e), "monotonic_s": now})
        unreal.unregister_slate_post_tick_callback(getattr(builtins, HANDLE))
        setattr(builtins, HANDLE, None)


setattr(builtins, HANDLE, unreal.register_slate_post_tick_callback(sample))
sample(0)
