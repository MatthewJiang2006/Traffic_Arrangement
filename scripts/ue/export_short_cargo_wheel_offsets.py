"""Read-only visible wheel locations relative to the test pawn."""
import json
from pathlib import Path
import unreal

root = Path(__file__).resolve().parents[2]
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
truck = next(a for a in actors if a.get_name() == "Car_0_0")
visual = next(a for a in actors if a.get_class().get_name() == "BP_Truck_C"
              and a.get_attach_parent_actor() == truck)
transform = truck.get_actor_transform()
wheels = []
for component in visual.get_components_by_class(unreal.StaticMeshComponent):
    if component.get_name().startswith("Wheel_"):
        p = transform.inverse_transform_location(component.get_world_location())
        wheels.append({"name": component.get_name(), "local_xyz_m": [p.x/100,p.y/100,p.z/100]})
(root/"runs/ue_scene/station_siting/W1_wheel_offsets.json").write_text(json.dumps(wheels,indent=2)+"\n")
