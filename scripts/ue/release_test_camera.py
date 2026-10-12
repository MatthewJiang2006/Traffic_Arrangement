"""Release diagnostic camera tracking and restore user-controlled PIE viewing."""
import builtins
import json
from pathlib import Path
import unreal

handle = getattr(builtins, 'traffic_test_camera_handle', None)
if handle is not None:
    unreal.unregister_slate_post_tick_callback(handle)
builtins.traffic_test_camera_handle = None
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
directors = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
             if 'CameraDirector' in a.get_class().get_name()]
for director in directors:
    director.set_actor_tick_enabled(True)
    director.set_mode(unreal.CameraDirectorMode.CAMERA_DIRECTOR_MODE_MANUAL)
result = {'tracking_callback_active': False, 'camera_directors': len(directors),
          'manual_camera_enabled': all(d.is_actor_tick_enabled() and
              d.get_mode() == unreal.CameraDirectorMode.CAMERA_DIRECTOR_MODE_MANUAL for d in directors)}
root = Path(__file__).resolve().parents[2]
(root/'runs/ue_scene/camera_release.json').write_text(json.dumps(result, indent=2)+'\n')
