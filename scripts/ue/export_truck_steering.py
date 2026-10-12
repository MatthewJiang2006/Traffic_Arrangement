"""Read current PIE truck wheel and steering properties without changing them."""
import json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
a=next(a for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name()
       for a in unreal.GameplayStatics.get_all_actors_of_class(w,unreal.Actor) if a.get_name()=='Car_5_1')
rows=[]
mesh=a.get_component_by_class(unreal.SkeletalMeshComponent)
bone_locations={}
for bone in ('WheelFL','WheelFR','WheelBL','WheelBR'):
    p=mesh.get_socket_location(bone)
    bone_locations[bone]=[p.x/100,p.y/100,p.z/100]
for c in a.get_components_by_class(unreal.ActorComponent):
    if 'VehicleMovement' not in c.get_class().get_name():continue
    row=dict(component=c.get_name(),class_name=c.get_class().get_name(),wheels=[])
    try:
        for setup in c.get_editor_property('wheel_setups'):
            cls=setup.get_editor_property('wheel_class');wheel=unreal.get_default_object(cls)
            props={}
            for name in ('steer_angle','shape_radius','shape_width','max_brake_torque'):
                try:props[name]=wheel.get_editor_property(name)
                except Exception as exc:props[name]=repr(exc)
            row['wheels'].append(dict(bone=str(setup.get_editor_property('bone_name')),properties=props))
    except Exception as exc:row['error']=repr(exc)
    try:row['steering_curve']=str(c.get_editor_property('steering_curve'))
    except Exception:pass
    rows.append(row)
(ROOT/'runs/ue_scene/truck_steering_properties.json').write_text(json.dumps(dict(vehicle=a.get_name(),components=rows,bone_locations_m=bone_locations,camera_changed=False,map_changed=False),indent=2),encoding='utf-8')
