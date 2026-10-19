import json
from pathlib import Path
import unreal
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
rows=[]
for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor):
 if a.get_class().get_name() != 'TrafficTruckPawn':continue
 ctl=a.get_controller();m=a.get_component_by_class(unreal.WheeledVehicleMovementComponent)
 rows.append({'name':a.get_name(),'controller':ctl.get_name() if ctl else None,'tick':a.is_actor_tick_enabled(), 'movement_active':m.is_active(),'movement_tick':m.is_component_tick_enabled(), 'gear':m.get_current_gear(), 'rpm':m.get_engine_rotation_speed()})
(Path(__file__).resolve().parents[2]/'runs/ue_scene/following_physics.json').write_text(json.dumps(rows))
