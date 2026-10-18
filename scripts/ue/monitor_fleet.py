"""One atomic fleet snapshot; body safety without repeated wheel diagnostics."""
import builtins,json,math,os,time
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
request=builtins.fleet_monitor_request
handle='flow_fleet_monitor'
if getattr(builtins,handle,None):unreal.unregister_slate_post_tick_callback(getattr(builtins,handle))
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
trucks={a.get_name():a for a in actors if a.get_name() in request['vehicles']}
visuals=[a for a in actors if a.get_class().get_name()=='BP_Truck_C']
meshes={n:a.get_component_by_class(unreal.SkeletalMeshComponent) for n,a in trucks.items()}
bodies={n:[c for v in visuals if v.get_attach_parent_actor()==a for c in v.get_components_by_class(unreal.StaticMeshComponent) if c.get_name() in {'Truck_Body_00','Truck_Body_01','TrafficShortCargo'}] for n,a in trucks.items()}
if set(trucks)!=set(request['vehicles']) or any(not b for b in bodies.values()):raise RuntimeError('Fleet monitor missing physical truck bodies')
origin=unreal.GameplayStatics.get_world_origin_location(world)
ignore=list(trucks.values())+visuals
path=ROOT/'runs/ue_scene'/request.get('file','flow_fleet_live.json')
history=Path(request['road_history']) if request.get('road_history') else None
if history:history.write_text('')
previous={};last=0;sequence=0

def absolute(p):return [(p.x+origin.x)/100,(p.y+origin.y)/100,(p.z+origin.z)/100]
def box(c):
    b=c.get_editor_property('static_mesh').get_bounds();tr=c.get_world_transform();scale=c.get_world_scale()
    return tr.transform_location(b.origin),unreal.Vector(b.box_extent.x*abs(scale.x),b.box_extent.y*abs(scale.y),b.box_extent.z*abs(scale.z)),c.get_world_rotation()
def overlap(a,b):
    pa,ea,ra=a;pb,eb,rb=b
    if abs(pa.z-pb.z)>=ea.z+eb.z:return False
    if abs(pa.x-pb.x)>ea.x+ea.y+eb.x+eb.y or abs(pa.y-pb.y)>ea.x+ea.y+eb.x+eb.y:return False
    angles=[math.radians(ra.yaw),math.radians(rb.yaw)]
    for t in (angles[0],angles[0]+math.pi/2,angles[1],angles[1]+math.pi/2):
        ax,ay=math.cos(t),math.sin(t)
        radius=sum(e.x*abs(math.cos(y)*ax+math.sin(y)*ay)+e.y*abs(-math.sin(y)*ax+math.cos(y)*ay) for e,y in zip((ea,eb),angles))
        if abs((pa.x-pb.x)*ax+(pa.y-pb.y)*ay)>=radius:return False
    return True
def publish(data):
    temporary=path.with_suffix('.tmp');temporary.write_text(json.dumps(data));os.replace(str(temporary),str(path))
def sample(delta):
    global last,sequence
    now=time.monotonic()
    if now-last<.05:return
    last=now;sequence+=1
    try:
        current={n:[(c,box(c)) for c in components] for n,components in bodies.items()}
        rows={};states={}
        for n,a in trucks.items():
            p=a.get_actor_location();r=a.get_actor_rotation();v=meshes[n].get_physics_linear_velocity();h=math.radians(r.yaw)
            xyz=absolute(p);states[n]=dict(x=xyz[0],y=xyz[1],z=xyz[2],heading=h,speed=(v.x*math.cos(h)+v.y*math.sin(h))/100,
                native_guard_active=a.get_editor_property('native_control_guard_active'),
                native_timed_controls_active=a.get_editor_property('native_timed_controls_active'),
                native_control_timeout_s=a.get_editor_property('native_control_timeout_seconds'),
                native_control_expired=a.get_editor_property('native_control_expired'),
                native_control_age_s=a.get_editor_property('native_control_age_seconds'),
                native_source_age_s=a.get_editor_property('native_control_source_age_seconds'),
                native_frame_s=a.get_editor_property('native_control_frame_seconds'),
                native_rejected_controls=a.get_editor_property('native_control_rejected_count'))
            row=dict(monotonic_s=now,truck_ue_m=xyz,bodies=[],wheels=[],pitch_deg=r.pitch,roll_deg=r.roll)
            for c,b in current[n]:
                centre,half,rotation=b;key=(n,c.get_name());old=previous.get(key)
                moved=old is None or math.sqrt((centre.x-old[0].x)**2+(centre.y-old[0].y)**2+(centre.z-old[0].z)**2)>.1 or abs((rotation.yaw-old[1]+180)%360-180)>.01
                hit_actor=old[2] if old else None
                if moved:
                    dyaw=math.radians(abs((rotation.yaw-(old[1] if old else rotation.yaw)+180)%360-180));inflation=math.hypot(half.x,half.y)*math.sin(dyaw/2)
                    swept=unreal.Vector(half.x+inflation,half.y+inflation,half.z)
                    hit=unreal.SystemLibrary.box_trace_single(world,old[0] if old else centre,centre,swept,rotation,unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
                    values=hit.to_tuple() if hit else None;hit_actor=values[9].get_name() if values and values[0] and values[9] else None
                other=next((name for name,boxes in current.items() if name!=n and any(overlap(b,candidate) for _,candidate in boxes)),None)
                row['bodies'].append(dict(name=c.get_name(),centre_ue_m=absolute(centre),half_size_m=[half.x/100,half.y/100,half.z/100],yaw_deg=rotation.yaw,static_sweep_hit=hit_actor,parked_vehicle_overlap=other))
                previous[key]=(centre,rotation.yaw,hit_actor)
            rows[n]=row
        publish(dict(monotonic_s=now,sequence=sequence,paused=unreal.GameplayStatics.is_game_paused(world),game_time_s=unreal.GameplayStatics.get_time_seconds(world),states=states,audits=rows))
        if history:
            data=dict(monotonic_s=now,game_time_s=unreal.GameplayStatics.get_time_seconds(world),
                positions={n:[s['x']-1.16425*math.cos(s['heading']),s['y']-1.16425*math.sin(s['heading']),s['heading'],s['speed']] for n,s in states.items()},
                rear_z_m={n:s['z'] for n,s in states.items()})
            with history.open('a') as stream:stream.write(json.dumps(data)+'\n')
    except Exception as exc:
        publish(dict(monotonic_s=now,error=repr(exc)));unreal.unregister_slate_post_tick_callback(getattr(builtins,handle));setattr(builtins,handle,None)
setattr(builtins,handle,unreal.register_slate_post_tick_callback(sample));sample(0)


def sample_wheels(name):
    visual=next(v for v in visuals if v.get_attach_parent_actor()==trucks[name])
    rows=[]
    for c in visual.get_components_by_class(unreal.StaticMeshComponent):
        if not c.get_name().startswith('Wheel_'):continue
        p=c.get_world_location();b=c.get_editor_property('static_mesh').get_bounds();tr=c.get_world_transform()
        bottom=min(tr.transform_location(unreal.Vector(b.origin.x+x*b.box_extent.x,b.origin.y+y*b.box_extent.y,b.origin.z+z*b.box_extent.z)).z for x in (-1,1) for y in (-1,1) for z in (-1,1))
        hit=unreal.SystemLibrary.line_trace_single(world,p+unreal.Vector(0,0,300),p-unreal.Vector(0,0,1000),unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
        values=hit.to_tuple() if hit else None
        rows.append(dict(name=c.get_name(),road_actor=values[9].get_name() if values and values[0] and values[9] else None,clearance_m=(bottom-values[5].z)/100 if values and values[0] else None))
    return dict(monotonic_s=time.monotonic(),vehicle=name,wheels=rows)
builtins.flow_fleet_sample_wheels=sample_wheels
