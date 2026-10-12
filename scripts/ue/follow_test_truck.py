"""Position the test camera once; leave subsequent viewing under user control."""
import builtins
import json
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
old = getattr(builtins, "traffic_test_camera_handle", None)
if old is not None:
    unreal.unregister_slate_post_tick_callback(old)
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
truck = next(a for a in actors if a.get_name() == "Car_0_0")
director = next(a for a in actors if "CameraDirector" in a.get_class().get_name())
camera = director.get_editor_property("ExternalCamera")
controller = unreal.GameplayStatics.get_player_controller(world, 0)
director.set_actor_tick_enabled(True)
director.set_mode(unreal.CameraDirectorMode.CAMERA_DIRECTOR_MODE_MANUAL)
component = camera.get_component_by_class(unreal.CameraComponent)
if component:
    component.set_field_of_view(65)
controller.set_view_target_with_blend(camera)


def follow(delta):
    target = truck.get_actor_location()+unreal.Vector(0, 0, 140)
    position = truck.get_actor_location()+unreal.Vector(-1200, -1600, 2200)
    rotation = unreal.MathLibrary.find_look_at_rotation(position, target)
    camera.set_actor_location_and_rotation(position, rotation, False, False)
    # Also covers an ejected PIE editor viewport.
    unreal.EditorLevelLibrary.set_level_viewport_camera_info(position, rotation)


builtins.traffic_test_camera_handle = None
follow(0)
(ROOT/"runs/ue_scene/test_camera_follow.json").write_text(json.dumps({"vehicle": "Car_0_0", "camera": camera.get_name(), "mode": "one-time camera positioning; user-controlled thereafter"})+"\n")
