"""Read-only body-box sweeps of access candidates; wheel height is diagnostic."""
import json
import math
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
request=json.loads((ROOT/'config/planning/ue_access_connection_candidates.json').read_text())
world=next(w for w in unreal.ObjectIterator(unreal.World)
           if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
ignore=[a for a in actors if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C','CameraActor')]
truck=next(a for a in actors if a.get_name()=='Car_0_0')
visual=next(a for a in actors if a.get_class().get_name()=='BP_Truck_C' and a.get_attach_parent_actor()==truck)
pawn_pose=truck.get_actor_transform();templates=[]
for c in visual.get_components_by_class(unreal.StaticMeshComponent):
    if c.get_name() not in ('Truck_Body_00','Truck_Body_01','TrafficShortCargo'):continue
    bounds=c.get_editor_property('static_mesh').get_bounds()
    centre=c.get_world_transform().transform_location(bounds.origin)
    local=pawn_pose.inverse_transform_location(centre);scale=c.get_world_scale()
    half=unreal.Vector(bounds.box_extent.x*abs(scale.x),bounds.box_extent.y*abs(scale.y),bounds.box_extent.z*abs(scale.z))
    templates.append((c.get_name(),local,half,c.get_world_rotation().yaw-truck.get_actor_rotation().yaw))
if len(templates)!=3:raise RuntimeError('Incomplete truck body geometry')
cases={}
for key,phases in request['cases'].items():
    hits=[];count=0
    for phase in phases:
        for a,b in zip(phase['centerline'],phase['centerline'][1:]):
            yaw=math.atan2(b[1]-a[1],b[0]-a[0])+(math.pi if phase['reverse'] else 0)
            co,si=math.cos(yaw),math.sin(yaw)
            steps=max(1,math.ceil(math.hypot(a[0]-b[0],a[1]-b[1])/.4))
            for k in range(steps+1):
                t=k/steps;x=a[0]+t*(b[0]-a[0])+1.16425*co;y=a[1]+t*(b[1]-a[1])+1.16425*si
                count+=1
                for name,local,half,relative_yaw in templates:
                    centre=unreal.Vector(x*100-origin.x+local.x*co-local.y*si,
                        y*100-origin.y+local.x*si+local.y*co,
                        89.4-origin.z+local.z)
                    result=unreal.SystemLibrary.box_trace_single(world,centre,centre,half,
                        unreal.Rotator(pitch=0,yaw=math.degrees(yaw)+relative_yaw,roll=0),
                        unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
                    values=result.to_tuple() if result else None
                    if values and values[0]:hits.append(dict(phase=phase['label'],body=name,actor=values[9].get_name(),xy_m=[x,y]))
    cases[key]=dict(status='PASS' if not hits else 'STATIC_OVERLAP',body_sweep_hits=hits,pose_count=count)
report=dict(status='PASS' if all(v['status']=='PASS' for v in cases.values()) else 'FAIL',cases=cases,
    scope='Static actual-body boxes at nominal chassis height; dynamic physics remains required',
    camera_changed=False,map_changed=False,source_graph_sha256=request['source_graph_sha256'])
(ROOT/'runs/ue_access_connections/static_precheck.json').write_text(json.dumps(report,indent=2)+'\n')
