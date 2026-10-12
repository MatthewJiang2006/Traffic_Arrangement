"""Record live articulated-trailer poses, road contact, hitch error and swept-body hits."""
import builtins
import json
import math
import os
from pathlib import Path
import time
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"runs/ue_scene/trailer_trial_live.json"
HISTORY = ROOT/"runs/ue_scene/trailer_trial_history.jsonl"
if getattr(builtins, "traffic_trailer_monitor", None) is not None:
    unreal.unregister_slate_post_tick_callback(builtins.traffic_trailer_monitor)
world = next(w for w in unreal.ObjectIterator(unreal.World) if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
trial_name = json.loads((ROOT/"runs/ue_scene/trailer_trial_request.json").read_text())["trailer_name"]
trailer = next(a for a in actors if a.get_name() == trial_name)
truck = next(a for a in actors if a.get_name() == "Car_0_0")
visual = next(a for a in actors if a.get_class().get_name() == "BP_Truck_C" and a.get_attach_parent_actor() == truck)
mesh = trailer.get_component_by_class(unreal.SkeletalMeshComponent)
visual_mesh = visual.get_component_by_class(unreal.SkeletalMeshComponent)
constraint = trailer.get_component_by_class(unreal.PhysicsConstraintComponent)
wheels = [c for c in trailer.get_components_by_class(unreal.StaticMeshComponent) if c.get_name().startswith("Wheel_")]
cargo = next(c for c in trailer.get_components_by_class(unreal.StaticMeshComponent) if c.get_name() == "Trailer")
frame = next(c for c in trailer.get_components_by_class(unreal.StaticMeshComponent) if c.get_name() == "TrailerBottom")
origin = unreal.GameplayStatics.get_world_origin_location(world)
all_vehicles = [a for a in actors if a.get_class().get_name() in ("TrafficTruckPawn", "BP_Truck_C")]
setup = json.loads((ROOT/"runs/ue_scene/trailer_trial_setup.json").read_text())
local_hitch = unreal.Vector(-447.0967407, 0, 100.1343689+setup["native_hitch_vertical_adjustment_cm"])
previous = None
previous_physics = None
last = 0


def publish(row):
    temporary = OUT.with_suffix(".tmp")
    temporary.write_text(json.dumps(row), encoding="utf-8")
    os.replace(temporary, OUT)


def absolute(v):
    return [round((v.x+origin.x)/100, 4), round((v.y+origin.y)/100, 4), round((v.z+origin.z)/100, 4)]


def sample(delta):
    global last, previous, previous_physics
    now = time.monotonic()
    if now-last < .1:
        return
    last = now
    try:
        if not unreal.SystemLibrary.is_valid(trailer):
            unreal.unregister_slate_post_tick_callback(builtins.traffic_trailer_monitor)
            builtins.traffic_trailer_monitor = None
            return
        rotation = mesh.get_world_rotation()
        transform = mesh.get_world_transform()
        truck_hitch = visual_mesh.get_socket_location("TruckAtt")
        trailer_hitch = transform.transform_location(local_hitch)
        row = {"monotonic_s": now, "game_time_s": unreal.GameplayStatics.get_time_seconds(world),
               "truck_ue_m": absolute(truck.get_actor_location()), "truck_yaw_deg": truck.get_actor_rotation().yaw,
               "trailer_ue_m": absolute(mesh.get_world_location()), "trailer_yaw_deg": rotation.yaw,
               "trailer_pitch_deg": rotation.pitch, "trailer_roll_deg": rotation.roll,
               "hitch_error_m": math.sqrt((truck_hitch.x-trailer_hitch.x)**2+(truck_hitch.y-trailer_hitch.y)**2+(truck_hitch.z-trailer_hitch.z)**2)/100,
               "constraint_broken": constraint.is_broken(), "physics": mesh.is_simulating_physics("Root"), "wheels": []}
        for c in wheels:
            p = c.get_world_location()
            bottom = p.z-54.6
            hit = unreal.SystemLibrary.line_trace_single(world, p+unreal.Vector(0, 0, 200), p-unreal.Vector(0, 0, 400),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, all_vehicles+[trailer], unreal.DrawDebugTrace.NONE, True)
            values = hit.to_tuple() if hit is not None else None
            road = values[9].get_name() if values and values[0] and values[9] else None
            row["wheels"].append({"name": c.get_name(), "ue_m": absolute(p), "road_actor": road,
                                  "clearance_m": (bottom-values[5].z)/100 if values and values[0] else None})
        bounds = cargo.get_editor_property("static_mesh").get_bounds()
        centre = cargo.get_world_transform().transform_location(bounds.origin)
        hit = unreal.SystemLibrary.box_trace_single(world, previous if previous is not None else centre,
            centre, bounds.box_extent, cargo.get_world_rotation(), unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
            False, [trailer, truck, visual], unreal.DrawDebugTrace.NONE, True)
        previous = centre
        values = hit.to_tuple() if hit is not None else None
        row["cargo_sweep_hit"] = values[9].get_name() if values and values[0] and values[9] else None
        row["cargo_ue_m"] = absolute(centre)
        frame_bounds = frame.get_editor_property("static_mesh").get_bounds()
        physics_centre = frame.get_world_transform().transform_location(frame_bounds.origin)
        physics_hit = unreal.SystemLibrary.box_trace_single(world,
            previous_physics if previous_physics is not None else physics_centre, physics_centre,
            frame_bounds.box_extent, frame.get_world_rotation(),
            unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, [trailer, truck, visual], unreal.DrawDebugTrace.NONE, True)
        previous_physics = physics_centre
        physics_values = physics_hit.to_tuple() if physics_hit is not None else None
        row["physics_sweep_hit"] = physics_values[9].get_name() if physics_values and physics_values[0] and physics_values[9] else None
        row["body_boxes"] = [
            {"centre_ue_m": absolute(centre), "yaw_deg": cargo.get_world_rotation().yaw,
             "half_xy_m": [bounds.box_extent.x/100, bounds.box_extent.y/100]},
            {"centre_ue_m": absolute(physics_centre), "yaw_deg": frame.get_world_rotation().yaw,
             "half_xy_m": [frame_bounds.box_extent.x/100, frame_bounds.box_extent.y/100]}]
        row["articulation_deg"] = (row["truck_yaw_deg"]-rotation.yaw+180)%360-180
        publish(row)
        with HISTORY.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(row)+"\n")
    except Exception as e:
        publish({"error": repr(e), "monotonic_s": now})
        unreal.unregister_slate_post_tick_callback(builtins.traffic_trailer_monitor)
        builtins.traffic_trailer_monitor = None


builtins.traffic_trailer_monitor = unreal.register_slate_post_tick_callback(sample)
sample(0)
