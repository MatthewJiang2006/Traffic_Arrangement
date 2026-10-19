"""PIE-only cargo appearance, grounded unload props and five-second cleanup."""
import builtins
import json
import os
import time
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
REQUEST = ROOT/'runs/ue_scene/cargo_visual_request.json'
request = json.loads(REQUEST.read_text())
old = getattr(builtins, 'traffic_cargo_visuals', None)
if old is not None:
    old['stop']()
builtins.traffic_cargo_visuals = None
if request.get('action') == 'STOP':
    Path(request['status_path']).write_text(json.dumps({'status': 'STOPPED', 'restored': True})+'\n')
else:
    world = next(w for w in unreal.ObjectIterator(unreal.World)
                 if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
    origin = unreal.GameplayStatics.get_world_origin_location(world)
    actors = unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
    components = {}
    for visual in actors:
        parent = visual.get_attach_parent_actor()
        if visual.get_class().get_name() == 'BP_Truck_C' and parent and parent.get_name() in request['vehicles']:
            components[parent.get_name()] = next(c for c in visual.get_components_by_class(unreal.StaticMeshComponent)
                                                 if c.get_name() == 'TrafficShortCargo')
    if set(components) != set(request['vehicles']):
        raise RuntimeError('Cargo visual components missing')
    original = {name: (c.is_visible(), c.get_editor_property('hidden_in_game')) for name, c in components.items()}
    pool={name:next(a for a in actors if a.get_name()==prop) for name,prop in request['prop_actors'].items()}
    for actor in pool.values():
        actor.set_actor_hidden_in_game(True)
        actor.get_component_by_class(unreal.StaticMeshComponent).set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
    state = dict(handle=None, seen=set(), props=[], events=[], last=0, stopped=False)
    status_path = Path(request['status_path'])
    def publish(status='RUNNING'):
        temporary = status_path.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(status=status, events=state['events'],
            attached_visible={name:c.is_visible() and not c.get_editor_property('hidden_in_game') for name,c in components.items()},
            ground_props=len(state['props']), camera_changed=False, map_saved=False))+'\n')
        os.replace(temporary, status_path)

    def visible(component, value):
        component.set_visibility(value, True)
        component.set_hidden_in_game(not value, True)

    def drop(command, now):
        cargo = components[command['vehicle']]
        pad = request['unloading_pads'][command['site']]
        x,y = pad['xy_m']
        centre = unreal.Vector(x*100-origin.x, y*100-origin.y, 0)
        hit = unreal.SystemLibrary.line_trace_single(world,
            unreal.Vector(centre.x,centre.y,400-origin.z), unreal.Vector(centre.x,centre.y,-100-origin.z),
            unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False,
            [a for a in actors if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C')],
            unreal.DrawDebugTrace.NONE, True)
        values = hit.to_tuple() if hit is not None else None
        if not values or not values[0]:raise RuntimeError('No unloading prop ground')
        if not .99<(values[5].z+origin.z)/100<1.16 or not values[9].get_name().startswith(('SM_Floor_','SM_Road_')):
            raise RuntimeError('Unloading prop must stand on actual floor, not an obstacle')
        mesh = cargo.get_editor_property('static_mesh')
        scale = cargo.get_world_scale()
        rotation = unreal.Rotator(pitch=0,yaw=pad.get('yaw_deg',180),roll=0)
        transform = unreal.Transform(location=unreal.Vector(0,0,0), rotation=rotation, scale=scale)
        bounds = mesh.get_bounds()
        offsets = [transform.transform_location(unreal.Vector(bounds.origin.x+i*bounds.box_extent.x,
                   bounds.origin.y+j*bounds.box_extent.y,bounds.origin.z+k*bounds.box_extent.z))
                   for i in (-1,1) for j in (-1,1) for k in (-1,1)]
        centre.z = values[5].z
        location = unreal.Vector(centre.x-(max(p.x for p in offsets)+min(p.x for p in offsets))/2,
                                 centre.y-(max(p.y for p in offsets)+min(p.y for p in offsets))/2,
                                 centre.z-min(p.z for p in offsets)+1)
        transform = unreal.Transform(location=location,rotation=rotation,scale=scale)
        half=unreal.Vector(*[(max(getattr(p,a) for p in offsets)-min(getattr(p,a) for p in offsets))/2 for a in ('x','y','z')])
        box_centre=unreal.Vector(centre.x,centre.y,centre.z+half.z+1)
        obstruction=unreal.SystemLibrary.box_trace_single(world,box_centre,box_centre,half,unreal.Rotator(),
            unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,list(pool.values()),unreal.DrawDebugTrace.NONE,True)
        obstruction_values=obstruction.to_tuple() if obstruction is not None else None
        if obstruction_values and obstruction_values[0]:
            raise RuntimeError('Unloading prop placement obstructed: '+obstruction_values[9].get_name())
        actor = pool[command['vehicle']]
        component = actor.get_component_by_class(unreal.StaticMeshComponent)
        component.set_mobility(unreal.ComponentMobility.MOVABLE)
        component.set_static_mesh(mesh)
        component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
        component.set_simulate_physics(False)
        actor.set_actor_transform(transform,False,True)
        actor.set_actor_hidden_in_game(False)
        visible(cargo,False)
        state['props'].append((actor,now+5,command))
        state['events'].append(dict(event='CARGO_DROPPED',id=command['id'],vehicle=command['vehicle'],
            monotonic_s=now,xy_m=[x,y],ground_actor=values[9].get_name(),
            bottom_z_m=(centre.z+origin.z+1)/100,ground_z_m=(centre.z+origin.z)/100))

    def stop():
        if state['stopped']:return
        state['stopped']=True
        if state['handle'] is not None:unreal.unregister_slate_post_tick_callback(state['handle'])
        for actor in pool.values():actor.set_actor_hidden_in_game(True)
        state['props'].clear()
        for name,c in components.items():
            c.set_visibility(original[name][0],True);c.set_hidden_in_game(original[name][1],True)
        publish('STOPPED')

    def tick(delta):
        now=time.monotonic()
        if now-state['last']<.1:return
        state['last']=now
        try:
            commands=json.loads(Path(request['events_path']).read_text())['commands']
            for command in commands:
                if command['id'] in state['seen']:continue
                if command['operation']=='LOADING':
                    visible(components[command['vehicle']],True)
                    state['events'].append(dict(event='CARGO_ATTACHED',id=command['id'],vehicle=command['vehicle'],monotonic_s=now))
                else:drop(command,now)
                state['seen'].add(command['id'])
            for actor,deadline,command in list(state['props']):
                if now>=deadline:
                    actor.set_actor_hidden_in_game(True);state['props'].remove((actor,deadline,command))
                    state['events'].append(dict(event='GROUND_CARGO_REMOVED',id=command['id'],vehicle=command['vehicle'],monotonic_s=now))
            publish()
        except Exception as exc:
            state['events'].append(dict(event='ERROR',error=repr(exc),monotonic_s=now))
            stop()
    for component in components.values():visible(component,False)
    builtins.traffic_cargo_visuals={'stop':stop}
    state['handle']=unreal.register_slate_post_tick_callback(tick)
    publish()
