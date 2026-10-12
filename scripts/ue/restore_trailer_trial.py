"""Restore the isolated towing pawn's normal chassis wheel/visual setup."""
import json
from pathlib import Path
import unreal

world = next(w for w in unreal.ObjectIterator(unreal.World) if "UEDPIE_" in w.get_path_name() and "IndustrialArea（复件）" in w.get_path_name())
truck = next(a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor) if a.get_name() == "Car_0_0")
result = {"restored": truck.configure_trailer_trial(False), "vehicle": "Car_0_0"}
(Path(__file__).resolve().parents[2]/"runs/ue_scene/trailer_trial_restored.json").write_text(json.dumps(result), encoding="utf-8")
