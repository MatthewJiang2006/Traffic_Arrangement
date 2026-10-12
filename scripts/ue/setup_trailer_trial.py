"""Connect one real BP_Trailer3_Child physics assembly to the AirSim test truck."""
import json
import math
from pathlib import Path
from statistics import mean
import unreal

ROOT = Path(__file__).resolve().parents[2]
world = next(w for w in unreal.ObjectIterator(unreal.World) if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
trial_name = json.loads((ROOT/"runs/ue_scene/trailer_trial_request.json").read_text())["trailer_name"]
trailer = next(a for a in actors if a.get_name() == trial_name)
truck = next(a for a in actors if a.get_name() == "Car_0_0")
visual = next(a for a in actors if a.get_class().get_name() == "BP_Truck_C" and a.get_attach_parent_actor() == truck)
trailer.set_actor_tick_enabled(False)
if not truck.configure_trailer_trial(True):
    raise RuntimeError("Could not configure the isolated towing chassis")
mesh = trailer.get_component_by_class(unreal.SkeletalMeshComponent)
truck_mesh = truck.get_component_by_class(unreal.SkeletalMeshComponent)
visual_mesh = visual.get_component_by_class(unreal.SkeletalMeshComponent)
constraint = trailer.get_component_by_class(unreal.PhysicsConstraintComponent)
constraint.break_constraint()
mesh.set_all_bodies_simulate_physics(False)
mesh.set_simulate_physics(False)
mesh.set_animation_mode(unreal.AnimationMode.ANIMATION_SINGLE_NODE)
for c in trailer.get_components_by_class(unreal.PrimitiveComponent):
    c.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
mesh.attach_to_component(trailer.root_component, "", unreal.AttachmentRule.KEEP_RELATIVE,
                         unreal.AttachmentRule.KEEP_RELATIVE, unreal.AttachmentRule.KEEP_RELATIVE, False)
mesh.set_relative_location(unreal.Vector(0, 0, 0), False, True)
mesh.set_relative_rotation(unreal.Rotator(0, 0, 0), False, True)
origin = unreal.GameplayStatics.get_world_origin_location(world)
hitch = visual_mesh.get_socket_location("TruckAtt")
yaw = truck.get_actor_rotation().yaw
rotation = unreal.Rotator(pitch=0, yaw=yaw, roll=0)
native_hitch = unreal.Vector(-447.0967407, 0, 100.1343689)
angle = math.radians(yaw)
offset = unreal.Vector(native_hitch.x*math.cos(angle), native_hitch.x*math.sin(angle), native_hitch.z)
location = hitch-offset
wheel_offsets = []
for c in trailer.get_components_by_class(unreal.StaticMeshComponent):
    if c.get_name().startswith("Wheel_"):
        # The stock spherical colliders have a 54.6 cm radius, while the tire
        # mesh has a 57.9 cm radius. Match the visible radius to its collider.
        c.set_relative_scale3d(unreal.Vector(54.6/58.49583435, 1, 54.6/57.90038681))
        wheel_offsets.append(c.get_world_location().z-54.6-mesh.get_world_location().z)
location.z = 110-origin.z-mean(wheel_offsets)
trailer.set_actor_location_and_rotation(location, rotation, False, True)
mesh.set_world_location_and_rotation(location, rotation, False, True)
constraint.set_world_location_and_rotation(hitch, rotation, False, True)
constraint.set_linear_x_limit(unreal.LinearConstraintMotion.LCM_LOCKED, 0)
constraint.set_linear_y_limit(unreal.LinearConstraintMotion.LCM_LOCKED, 0)
constraint.set_linear_z_limit(unreal.LinearConstraintMotion.LCM_LOCKED, 0)
# X points along the truck: free Z yaw, limited Y pitch and X roll.
constraint.set_angular_swing1_limit(unreal.AngularConstraintMotion.ACM_FREE, 0)
constraint.set_angular_swing2_limit(unreal.AngularConstraintMotion.ACM_LIMITED, 15)
constraint.set_angular_twist_limit(unreal.AngularConstraintMotion.ACM_LIMITED, 5)
constraint.set_disable_collision(True)
constraint.set_linear_breakable(False, 0)
constraint.set_angular_breakable(False, 0)
if not truck.fit_trailer_trial_collision(trailer):
    raise RuntimeError("Could not fit the selected trailer's collision envelope")
mesh.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
mesh.set_collision_profile_name("PhysicsActor")
mesh.set_simulate_physics(True)
mesh.set_all_bodies_simulate_physics(True)
mesh.set_all_physics_linear_velocity(unreal.Vector(0, 0, 0), False)
mesh.set_all_physics_angular_velocity_in_degrees(unreal.Vector(0, 0, 0), False)
constraint.set_constrained_components(truck_mesh, "ChassisBone", mesh, "Root")
result = {"vehicle": truck.get_name(), "trailer": trailer.get_name(), "blueprint": "BP_Trailer3_Child",
          "physics_enabled": mesh.is_simulating_physics(), "wheel_count": len(wheel_offsets),
          "hitch_from_origin_cm": [hitch.x, hitch.y, hitch.z],
          "trailer_mesh_from_origin_cm": [location.x, location.y, location.z],
          "native_hitch_vertical_adjustment_cm": hitch.z-location.z-native_hitch.z,
          "visual_tire_radius_cm": 54.6,
          "trial_air_sim_wheel_radius_increase_cm": 10,
          "trial_truck_visual_relative_z_cm": 11,
          "connection": "PhysX constraint to AirSim ChassisBone; seven-body transient PhysicsAsset fitted to child cargo/frame",
          "collision_envelope": "two Root boxes fitted to actual Trailer/TrailerBottom; original wheels/constraints retained",
          "truck_mass_kg": truck_mesh.get_mass(), "trailer_mass_kg": mesh.get_mass()}
(ROOT/"runs/ue_scene/trailer_trial_setup.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
