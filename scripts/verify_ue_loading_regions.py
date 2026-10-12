#!/usr/bin/env python3
"""Compare static scene snapshots over actual previous loading trial sweeps."""
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
RAW=ROOT/"runs/ue_scene/station_siting"
before=json.loads((RAW/"scene_before_middle_service_ramps.json").read_text())
after=json.loads((ROOT/"runs/ue_scene/scene.json").read_text())
maps=Path("/home/song03/Traffic_ue4/Content/IndustrialArea/Map")
report=dict(before_map_sha256={},after_map_sha256={},regions={},read_only=True,
            scope="Static actor mesh, pose and bounds snapshot over recorded loading trial sweeps")
for name,backup in (("IndustrialArea（复件）.umap","IndustrialArea_before_middle_service_ramps.umap"),
                    ("IndustrialArea_Copy.umap","IndustrialArea_Copy_before_middle_service_ramps.umap")):
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


for site in ("W1","W2"):
    rectangles=[]
    for case in ("reservation","straight_south","straight_north"):
        source=json.loads((RAW/f"loading_access_{site}_{case}.json").read_text())
        if source["map_sha256"]!=report["before_map_sha256"]:
            raise RuntimeError("Before-map backup differs from recorded trial: "+site)
        for phase in source["phases"].values():
            points=phase["path_xy_heading"]
            rectangles += [[min(a[0],b[0])-5,min(a[1],b[1])-5,
                            max(a[0],b[0])+5,max(a[1],b[1])+5] for a,b in zip(points,points[1:])]
    old,new=selected(before,rectangles),selected(after,rectangles)
    digest=lambda value:hashlib.sha256(json.dumps(value,sort_keys=True).encode()).hexdigest()
    report["regions"][site]=dict(sweep_rectangles_m=rectangles,static_actor_count=len(new),
        before_geometry_sha256=digest(old),after_geometry_sha256=digest(new),unchanged=old==new)
    print(site,"unchanged static geometry:",old==new)
(RAW/"loading_regions_after_middle_ramps.json").write_text(json.dumps(report,indent=2)+"\n")
