"""Read the loaded native pavement mesh; never modify actors or assets."""
import json
from pathlib import Path
import unreal


def export():
    output=Path(__file__).resolve().parents[2]/'runs/ue_scene/drivable_surface.json'
    try:
        world=unreal.EditorLevelLibrary.get_game_world()
        cars=[a for a in unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
              if a.get_class().get_name()=='TrafficTruckPawn']
        if not cars:raise RuntimeError('Native traffic truck unavailable')
        row=json.loads(cars[0].export_drivable_triangles())
        output.write_text(json.dumps(row),encoding='utf-8')
    except Exception as error:
        output.write_text(json.dumps(dict(error=repr(error))),encoding='utf-8')
        raise


export()
