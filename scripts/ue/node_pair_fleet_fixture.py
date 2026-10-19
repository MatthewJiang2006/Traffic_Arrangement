"""Reversible freeze/hide/staging of inactive node-pair test vehicles."""
import builtins,json
from pathlib import Path
import unreal
ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'runs/ue_node_pairs'
request=json.loads((OUT/'fleet_request.json').read_text())
OUT=Path(request.get('output_dir',str(OUT)))
OUT.mkdir(parents=True,exist_ok=True)
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'map.map' in w.get_path_name())
actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
state=getattr(builtins,'node_pair_fleet_fixture',None)
if request['action']=='INIT':
    if state is not None:raise RuntimeError('Pair fixture already active')
    state={}
    for a in actors:
        if a.get_class().get_name()!='TrafficTruckPawn':continue
        mesh=a.get_component_by_class(unreal.SkeletalMeshComponent)
        if mesh is None:continue
        children=[a]+[v for v in actors if v.get_attach_parent_actor()==a]
        state[a.get_name()]=dict(actor=a,mesh=mesh,physics=mesh.is_simulating_physics(),
            visibility=[(v,v.get_editor_property('hidden')) for v in children],
            collisions=[(v,v.get_actor_enable_collision()) for v in children],pose=a.get_actor_transform())
    builtins.node_pair_fleet_fixture=state
    builtins.node_pair_original_time_dilation=unreal.GameplayStatics.get_global_time_dilation(world)
    physics=unreal.get_default_object(unreal.load_class(None,'/Script/Engine.PhysicsSettings'))
    builtins.node_pair_physics_settings=(physics,{k:physics.get_editor_property(k) for k in
        ('MaxPhysicsDeltaTime','bSubstepping','MaxSubstepDeltaTime','MaxSubsteps')})
    scale=float(request.get('time_scale',1))
    if scale>1 or request.get('physics_substepping',False):
        physics.set_editor_property('bSubstepping',True)
        physics.set_editor_property('MaxSubstepDeltaTime',1/120)
        physics.set_editor_property('MaxSubsteps',16)
        physics.set_editor_property('MaxPhysicsDeltaTime',max(scale/30,16/120 if request.get('physics_substepping',False) else 0.))
    unreal.GameplayStatics.set_global_time_dilation(world,float(request.get('time_scale',1)))
    names=list(state)
else:names=request.get('vehicles',list(state))
origin=unreal.GameplayStatics.get_world_origin_location(world)
for name in names:
    row=state[name];mesh=row['mesh']
    if request['action'] in ('INIT','FREEZE'):
        mesh.set_all_physics_linear_velocity(unreal.Vector(0,0,0),False)
        mesh.set_all_physics_angular_velocity_in_degrees(unreal.Vector(0,0,0),False)
        mesh.set_simulate_physics(False)
        for a,hidden in row['visibility']:a.set_actor_hidden_in_game(True)
        for a,enabled in row['collisions']:a.set_actor_enable_collision(False)
    elif request['action']=='ACTIVATE':
        for a,enabled in row['collisions']:a.set_actor_enable_collision(enabled)
        mesh.set_simulate_physics(row['physics'])
        for a,hidden in row['visibility']:a.set_actor_hidden_in_game(hidden)
    elif request['action']=='RESTORE':
        # Resetting an active substepped chassis can otherwise produce an
        # artificial impulse while restoring its world transform.
        mesh.set_all_physics_linear_velocity(unreal.Vector(0,0,0),False)
        mesh.set_all_physics_angular_velocity_in_degrees(unreal.Vector(0,0,0),False)
        mesh.set_simulate_physics(False)
        for a,enabled in row['collisions']:a.set_actor_enable_collision(False)
        row['actor'].set_actor_transform(row['pose'],False,True)
        for a,enabled in row['collisions']:a.set_actor_enable_collision(enabled)
        mesh.set_simulate_physics(row['physics'])
        for a,hidden in row['visibility']:a.set_actor_hidden_in_game(hidden)
if request['action']=='RESTORE':
    saved=getattr(builtins,'node_pair_physics_settings',None)
    if saved:
        physics,settings=saved
        for k,v in settings.items():physics.set_editor_property(k,v)
    unreal.GameplayStatics.set_global_time_dilation(world,builtins.node_pair_original_time_dilation)
    builtins.node_pair_fleet_fixture=None
(OUT/'fleet_status.json').write_text(json.dumps(dict(status='PASS',action=request['action'],vehicles=names,camera_changed=False,map_changed=False))+'\n')
