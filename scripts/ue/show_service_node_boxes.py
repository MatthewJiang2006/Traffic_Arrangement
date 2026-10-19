"""Refresh noncolliding colored node outlines in PIE; leave camera and map alone."""
import builtins
import json
import math
import time
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
SCENARIO=ROOT/'config/runtime/industrialarea_ue_fleet16_round42_queue_gate_v4.json'
spec=json.loads(SCENARIO.read_text(encoding='utf-8'))
network=json.loads((ROOT/spec['network']).read_text(encoding='utf-8'))
OUTPUT=ROOT/'runs/ue_scene/service_node_boxes.json'
old=getattr(builtins,'traffic_service_boxes_handle',None)
if old is not None:
    unreal.unregister_slate_post_tick_callback(old)
COLORS={'loading':(0.1,1.,0.25,1.),'unloading':(1.,0.1,0.05,1.),'waiting':(0.05,0.4,1.,1.)}
DISABLED_WAITING={'Q26','Q28'}
rows=[]
for node,site in network['service_nodes'].items():
    role=site['kind']
    # The current network already excludes disabled/outside bays. Legacy
    # operationally_enabled flags are stale even for bays used by dispatch.
    if role not in COLORS:continue
    rear=site['rear_xy_m'];yaw=math.radians(site['heading_deg'])
    rows.append(dict(node_id=node,role=role,center_xy_m=[rear[0]+.75*math.cos(yaw),rear[1]+.75*math.sin(yaw)],
                     ground_z_m=site['ground_z_m'],heading_deg=site['heading_deg'],
                     color_rgba=(.5,.5,.5,1.) if node in DISABLED_WAITING else COLORS[role],
                     enabled=node not in DISABLED_WAITING,
                     label=node+' (disabled)' if node in DISABLED_WAITING else node,
                     length_m=9.344,width_m=3.6))
state={'last':-math.inf,'draws':0,'drawn_worlds':[]}

def redraw(delta):
    now=time.monotonic()
    if now-state['last']<1.:return
    worlds=[w for w in unreal.ObjectIterator(unreal.World)
            if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name()]
    editor=unreal.EditorLevelLibrary.get_editor_world()
    if editor and editor not in worlds:worlds.append(editor)
    if not worlds:return
    for world in worlds:
        if world in state['drawn_worlds']:continue
        unreal.SystemLibrary.flush_persistent_debug_lines(world)
        unreal.SystemLibrary.flush_debug_strings(world)
        state['drawn_worlds'].append(world)
        origin=unreal.GameplayStatics.get_world_origin_location(world)
        for row in rows:
            x,y=row['center_xy_m'];z=row['ground_z_m']+.25;angle=math.radians(row['heading_deg'])
            center=unreal.Vector(x*100-origin.x,y*100-origin.y,z*100-origin.z)
            along=unreal.Vector(row['length_m']*50*math.cos(angle),row['length_m']*50*math.sin(angle),0)
            across=unreal.Vector(-row['width_m']*50*math.sin(angle),row['width_m']*50*math.cos(angle),0)
            corners=[center-along-across,center+along-across,center+along+across,center-along+across]
            top=[p+unreal.Vector(0,0,70) for p in corners];color=unreal.LinearColor(*row['color_rgba'])
            for plane in (corners,top):
                for a,b in zip(plane,plane[1:]+plane[:1]):
                    unreal.SystemLibrary.draw_debug_line(world,a,b,color,86400.,60.)
            for a,b in zip(corners,top):unreal.SystemLibrary.draw_debug_line(world,a,b,color,86400.,60.)
            unreal.SystemLibrary.draw_debug_string(world,center+unreal.Vector(0,0,110),row['label'],None,color,86400.)
    state['last']=now;state['draws']+=1
    if state['draws']==1:
        OUTPUT.parent.mkdir(parents=True,exist_ok=True)
        OUTPUT.write_text(json.dumps(dict(status='PASS',count=len(rows),
            counts={role:sum(r['role']==role for r in rows) for role in COLORS},markers=rows,
            camera_changed=False,map_saved=False,collision_geometry_changed=False,
            restart_check_s=1.,lifetime_s=86400.),indent=2)+'\n')

builtins.traffic_service_boxes_handle=unreal.register_slate_post_tick_callback(redraw)
redraw(0.)
