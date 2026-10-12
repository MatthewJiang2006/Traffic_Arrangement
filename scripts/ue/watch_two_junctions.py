"""Runtime overview of both junctions and the four diagnostic vehicles."""
import builtins
import unreal
old=getattr(builtins,'traffic_test_camera_handle',None)
if old is not None:
    unreal.unregister_slate_post_tick_callback(old)
    builtins.traffic_test_camera_handle=None
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
director=next(a for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor) if 'CameraDirector' in a.get_class().get_name())
camera=director.get_editor_property('ExternalCamera');director.set_actor_tick_enabled(True);director.set_mode(unreal.CameraDirectorMode.CAMERA_DIRECTOR_MODE_MANUAL)
position=unreal.Vector(500-origin.x,24400-origin.y,11000-origin.z)
rotation=unreal.Rotator(pitch=-90,yaw=0,roll=0)
camera.set_actor_location_and_rotation(position,rotation,False,False)
unreal.GameplayStatics.get_player_controller(world,0).set_view_target_with_blend(camera)
unreal.EditorLevelLibrary.set_level_viewport_camera_info(position,rotation)
