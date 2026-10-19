"""Inspect an isolated BP_Trailer3_Child probe and the current truck hitch."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
probe = next(a for a in actors if a.get_name() == "Traffic_Trailer_Probe")
probe.set_actor_tick_enabled(False)
truck = next(a for a in actors if a.get_name() == "Car_0_0")
visual = next(a for a in actors if a.get_class().get_name() == "BP_Truck_C"
              and a.get_attach_parent_actor() == truck)


def vector(v):
    return [v.x, v.y, v.z]


def describe(actor):
    rows = []
    for c in actor.get_components_by_class(unreal.SceneComponent):
        row = {"name": c.get_name(), "class": c.get_class().get_name(),
               "world_cm": vector(c.get_world_location()), "relative_cm": vector(c.get_editor_property("relative_location"))}
        if isinstance(c, unreal.PrimitiveComponent):
            row["simulating_physics"] = c.is_simulating_physics()
            row["collision"] = str(c.get_collision_enabled())
        if isinstance(c, unreal.StaticMeshComponent):
            m = c.get_editor_property("static_mesh")
            if m:
                row["mesh"] = m.get_path_name()
                b = m.get_bounds()
                row["mesh_origin_cm"] = vector(b.origin)
                row["mesh_extent_cm"] = vector(b.box_extent)
        if isinstance(c, unreal.SkeletalMeshComponent):
            row["sockets"] = {str(s): vector(c.get_socket_location(s)) for s in c.get_all_socket_names()}
        if isinstance(c, unreal.PhysicsConstraintComponent):
            row["constraint_methods"] = [n for n in dir(c) if any(s in n for s in ("angular", "linear", "constrain"))]
        rows.append(row)
    origin, extent = actor.get_actor_bounds(False)
    return {"actor": actor.get_path_name(), "root": actor.root_component.get_name(),
            "position_cm": vector(actor.get_actor_location()), "bounds_origin_cm": vector(origin),
            "bounds_extent_cm": vector(extent), "components": rows}


result = {"trailer": describe(probe), "truck": describe(truck), "truck_visual": describe(visual)}
(ROOT/"runs/ue_scene/trailer_probe_inspection.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
