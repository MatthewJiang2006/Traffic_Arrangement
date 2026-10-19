"""Overlap rotated ramp ends by 20 cm to prevent tire-sized seam gaps."""
import json
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/"runs/ue_scene/station_siting"
SITES = [("W1","IndustrialArea（复件）","IndustrialArea_before_loading_taper_overlap.umap"),
         ("W2","IndustrialArea_Copy","IndustrialArea_Copy_before_loading_taper_overlap.umap")]
handle = None


def apply(delta):
    global handle
    if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
        return
    if handle is not None:
        unreal.unregister_slate_post_tick_callback(handle)
        handle = None
    world = unreal.EditorLevelLibrary.get_editor_world()
    if world is None or world.get_name() != "IndustrialArea（复件）":
        raise RuntimeError("Wrong editor map")
    if unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages():
        raise RuntimeError("Unsaved map edits")
    for _,_,backup in SITES:
        if not (OUT/backup).exists():
            raise RuntimeError("Both map backups required")
    actors = unreal.EditorLevelLibrary.get_all_level_actors()
    saved = []
    for site,level,_ in SITES:
        if not unreal.EditorLevelLibrary.set_current_level_by_name(level):
            raise RuntimeError("Cannot select "+level)
        for side in ("LowY","HighY"):
            tag = f"Traffic{site}LaneTaper{side}"
            hits = [a for a in actors if tag in [str(t) for t in a.tags]]
            if len(hits) != 1:
                raise RuntimeError("Expected one "+tag)
            hits[0].set_actor_scale3d(unreal.Vector(4.5,4.4,.04))
            saved.append(tag)
        if not unreal.EditorLevelLibrary.save_current_level():
            raise RuntimeError("Cannot save "+level)
    unreal.EditorLevelLibrary.set_current_level_by_name("IndustrialArea（复件）")
    (OUT/"loading_taper_overlap.json").write_text(json.dumps(dict(
        saved=True,actors=saved,size_m=[4.5,4.4,.04],overlap_m=.2),indent=2)+"\n")


if any("UEDPIE_" in w.get_path_name() for w in unreal.ObjectIterator(unreal.World)):
    handle = unreal.register_slate_post_tick_callback(apply)
    unreal.EditorLevelLibrary.editor_end_play()
else:
    apply(0)
