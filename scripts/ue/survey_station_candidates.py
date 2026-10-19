"""Read-only parking-envelope/ground survey for warehouse siting approval."""
import json,math,time
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/ue_scene/station_siting';OUT.mkdir(parents=True,exist_ok=True)
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
ignore=[a for a in actors if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C','CameraActor')]
areas={'east_lower':[25,34,54,79],'inner_lower':[-38,-20,99,124],
       'central_west':[-80,-20,171,186],'east_upper':[25,34,182,207],
       'inner_upper':[-38,-20,227,252],'east_middle_lower':[20,34,98,127],
       'east_middle_upper':[20,34,226,255]}
request_path=OUT/'request.json'
request=json.loads(request_path.read_text()) if request_path.exists() else {}
areas=request.get('areas',areas)
headings=request.get('headings',[0,90])

def ground(x,y):
    a=unreal.Vector(x*100-origin.x,y*100-origin.y,1200-origin.z)
    b=unreal.Vector(a.x,a.y,-100-origin.z)
    hit=unreal.SystemLibrary.line_trace_single(world,a,b,unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
    row=hit.to_tuple() if hit else None
    return ((row[5].z+origin.z)/100,row[9].get_name() if row[9] else '') if row and row[0] else (None,'')

rows=[];begin=time.monotonic()
for area,(x0,x1,y0,y1) in areas.items():
    for x in range(x0,x1+1,2):
        for y in range(y0,y1+1,2):
            z,surface=ground(x,y)
            for yaw in headings:
                row={'area':area,'xy_m':[x,y],'heading_deg':yaw,'surface_actor':surface,'ground_z_m':z,'clear':False}
                if z is None or not .7<z<1.35:
                    row['reason']='no low-level ground';rows.append(row);continue
                co,si=math.cos(math.radians(yaw)),math.sin(math.radians(yaw))
                values=[]
                for dx,dy in [(-4.5,-1.65),(-4.5,1.65),(4.5,-1.65),(4.5,1.65)]:
                    h,name=ground(x+dx*co-dy*si,y+dx*si+dy*co)
                    values.append(h)
                if any(h is None or abs(h-z)>.08 for h in values):
                    row['reason']='uneven or obstructed footprint ground';rows.append(row);continue
                center=unreal.Vector(x*100-origin.x,y*100-origin.y,(z+2.3)*100-origin.z)
                hit=unreal.SystemLibrary.box_trace_single(world,center,center,unreal.Vector(450,165,195),
                    unreal.Rotator(pitch=0,yaw=yaw,roll=0),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
                result=hit.to_tuple() if hit else None
                if result and result[0]:row['reason']='static overlap';row['obstacle_actor']=result[9].get_name() if result[9] else ''
                else:row['clear']=True
                rows.append(row)
result={'coordinate_frame':'UE absolute world metres','world_origin_cm':[origin.x,origin.y,origin.z],
        'parking_envelope_m':[9.,3.3,3.9],'read_only':True,'elapsed_s':time.monotonic()-begin,'areas':areas,'rows':rows}
(OUT/request.get('output','ground_clearance.json')).write_text(json.dumps(result,indent=2)+'\n')
