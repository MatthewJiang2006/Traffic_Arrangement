"""Read-only survey of full truck footprints inside the park perimeter."""
import json, math, time
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT/'runs/ue_scene/station_siting/internal_waiting_survey.json'
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
ignore = [a for a in actors if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C','CameraActor')]

def ground(x,y):
    p = unreal.Vector(x*100-origin.x,y*100-origin.y,1200-origin.z)
    hit = unreal.SystemLibrary.line_trace_single(world,p,unreal.Vector(p.x,p.y,-100-origin.z),
        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
    r = hit.to_tuple() if hit else None
    return ((r[5].z+origin.z)/100,r[9].get_name() if r[9] else '') if r and r[0] else (None,'')

rows=[];begin=time.monotonic()
for x in range(-80,49,4):
    for y in range(55,304,4):
        z,surface=ground(x,y)
        if z is None or not .7<z<1.35:continue
        for yaw in (0,90):
            co,si=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
            corners=[(x+dx*co-dy*si,y+dx*si+dy*co) for dx,dy in ((-4.5,-1.65),(-4.5,1.65),(4.5,-1.65),(4.5,1.65))]
            if any(not (-104<=px<=50 and 49<=py<=310) for px,py in corners):continue
            p=unreal.Vector(x*100-origin.x,y*100-origin.y,(z+2.35)*100-origin.z)
            hit=unreal.SystemLibrary.box_trace_single(world,p,p,unreal.Vector(450,165,195),
                unreal.Rotator(pitch=0,yaw=yaw,roll=0),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,
                False,ignore,unreal.DrawDebugTrace.NONE,True)
            r=hit.to_tuple() if hit else None
            if r and r[0]:continue
            heights=[ground(px,py)[0] for px,py in corners]
            if any(h is None or abs(h-z)>.15 for h in heights):continue
            rows.append(dict(xy_m=[x,y],heading_deg=yaw,ground_z_m=z,surface_actor=surface))
OUT.write_text(json.dumps(dict(read_only=True,camera_changed=False,map_saved=False,
    perimeter_m=[-104,49,50,310],footprint_m=[9,3.3],elapsed_s=time.monotonic()-begin,
    rows=rows),indent=2)+'\n')
