#!/usr/bin/env python3
"""Preserve prior access evidence only for unchanged recorded sweep geometry."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/"runs/ue_scene/station_siting"
before=json.loads((RAW/"scene_before_remaining_waiting_access.json").read_text())
after=json.loads((ROOT/"runs/ue_scene/scene.json").read_text())
maps=Path("/home/song03/Traffic_ue4/Content/IndustrialArea/Map")
report=dict(before_map_sha256={},after_map_sha256={},cases={},read_only=True)
for name,backup in (("IndustrialArea（复件）.umap","IndustrialArea_before_remaining_waiting_access.umap"),
                    ("IndustrialArea_Copy.umap","IndustrialArea_Copy_before_remaining_waiting_access.umap")):
    report["before_map_sha256"][name]=hashlib.sha256((RAW/backup).read_bytes()).hexdigest()
    report["after_map_sha256"][name]=hashlib.sha256((maps/name).read_bytes()).hexdigest()


def selected(scene,rectangles):
    result=[]
    for actor in scene["actors"]:
        if not actor["meshes"] or actor["class"] in ("TrafficTruckPawn","BP_Truck_C","CameraActor"):
            continue
        x,y,_=[v/100 for v in actor["bounds_origin_cm"]]
        hx,hy,_=[v/100 for v in actor["bounds_extent_cm"]]
        if any(x+hx>=a and x-hx<=c and y+hy>=b and y-hy<=d for a,b,c,d in rectangles):
            result.append(actor)
    return sorted(result,key=lambda a:a["path"])


for path in [*RAW.glob("service_access_*.json"),*RAW.glob("waiting_access_*.json")]:
    source=json.loads(path.read_text())
    if source.get("status")!="PASS" or source.get("map_sha256")!=report["before_map_sha256"]:
        continue
    rectangles=[]
    for phase in source["phases"].values():
        points=phase["path_xy_heading"]
        rectangles += [[min(a[0],b[0])-6,min(a[1],b[1])-6,
                        max(a[0],b[0])+6,max(a[1],b[1])+6] for a,b in zip(points,points[1:])]
    old,new=selected(before,rectangles),selected(after,rectangles)
    digest=lambda value:hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
    report["cases"][path.stem]=dict(static_actor_count=len(new),before_geometry_sha256=digest(old),
        after_geometry_sha256=digest(new),unchanged=old==new,sweep_rectangles_m=rectangles)
    print(path.stem,"unchanged:",old==new)
(RAW/"previous_access_regions_after_waiting_patch.json").write_text(json.dumps(report,indent=2)+"\n")
