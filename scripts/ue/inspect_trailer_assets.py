"""Read truck/trailer asset dimensions and Blueprint attachment-related defaults."""

import json
from pathlib import Path

import unreal


ROOT = Path(__file__).resolve().parents[2]
registry = unreal.AssetRegistryHelpers.get_asset_registry()
rows = []
for folder in ("/Game/TruckAndTrailers/Meshes", "/Game/TruckAndTrailers/Blueprints",
               "/Game/IndustrialArea/Meshes"):
    for data in registry.get_assets_by_path(folder, recursive=True):
        name = str(data.asset_name)
        if "Trailer" not in name and name not in ("BP_Truck", "ST_Truck_Body", "ST_Truck_Body_01"):
            continue
        asset = data.get_asset()
        row = {"name": name, "path": asset.get_path_name(), "class": asset.get_class().get_name()}
        if isinstance(asset, unreal.StaticMesh):
            bounds = asset.get_bounds()
            row["size_m"] = [round(v/50, 3) for v in (bounds.box_extent.x, bounds.box_extent.y, bounds.box_extent.z)]
            row["bounds_origin_cm"] = [bounds.origin.x, bounds.origin.y, bounds.origin.z]
        elif isinstance(asset, unreal.Blueprint):
            cls = unreal.load_class(None, asset.get_path_name()+"_C")
            if cls:
                default = unreal.get_default_object(cls)
                row["attachment_properties"] = {}
                for prop in dir(default):
                    if any(word in prop.lower() for word in ("trailer", "constraint", "attach", "hitch")):
                        try:
                            value = default.get_editor_property(prop)
                        except Exception:
                            continue
                        row["attachment_properties"][prop] = str(value)
                mesh = (default.get_component_by_class(unreal.SkeletalMeshComponent)
                        if isinstance(default, unreal.Actor) else None)
                if mesh:
                    row["skeletal_sockets"] = [str(v) for v in mesh.get_all_socket_names()]
        rows.append(row)
output = ROOT/"runs/ue_scene/trailer_assets.json"
output.write_text(json.dumps(rows, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
unreal.log("Trailer asset inventory written to "+str(output))
