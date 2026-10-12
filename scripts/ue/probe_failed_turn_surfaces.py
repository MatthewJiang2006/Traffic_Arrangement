"""Read-only ground survey at the failed left-turn approaches."""
import json
from pathlib import Path
import unreal

ROOT=Path(__file__).resolve().parents[2]
world=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin=unreal.GameplayStatics.get_world_origin_location(world)
actors=unreal.GameplayStatics.get_all_actors_of_class(world,unreal.Actor)
ignore=[a for a in actors if a.get_class().get_name() in ('TrafficTruckPawn','BP_Truck_C')]
rows=[]
for ys in ((213,214,215,215.5,216,217,218,219),(133,134,135,136,137,138,139,140)):
    for y in ys:
        hits=[]
        for step in range(37):
            x=-96+step*.5
            hit=unreal.SystemLibrary.line_trace_single(world,
                unreal.Vector(x*100-origin.x,y*100-origin.y,400-origin.z),
                unreal.Vector(x*100-origin.x,y*100-origin.y,-200-origin.z),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1,False,ignore,unreal.DrawDebugTrace.NONE,True)
            v=hit.to_tuple() if hit else None
            hits.append(dict(x=x,z=(v[5].z+origin.z)/100 if v and v[0] else None,actor=v[9].get_name() if v and v[0] and v[9] else None))
        rows.append(dict(y=y,hits=hits))
(ROOT/'runs/ue_node_pairs/failed_turn_surfaces.json').write_text(json.dumps(dict(rows=rows,map_changed=False,camera_changed=False),indent=2)+'\n')
