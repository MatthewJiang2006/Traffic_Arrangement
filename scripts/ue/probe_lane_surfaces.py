"""Read-only transverse surface survey; road mesh bounds include sidewalks."""
import json
from pathlib import Path
import unreal

ROOT = Path(__file__).resolve().parents[2]
world = next(w for w in unreal.ObjectIterator(unreal.World)
             if 'UEDPIE_' in w.get_path_name() and 'IndustrialArea（复件）' in w.get_path_name())
origin = unreal.GameplayStatics.get_world_origin_location(world)
ignore = [a for a in unreal.GameplayStatics.get_all_actors_of_class(world, unreal.Actor)
          if a.get_class().get_name() in ('TrafficTruckPawn', 'BP_Truck_C')]
survey = json.loads((ROOT/'config/topology/industrialarea_ue_survey.json').read_text(encoding='utf-8'))
rows = []
for column in (0, 1):
    centre_x = survey['junctions'][f'j_2_{column}']['center_m'][0]
    for y in (180, 198, 205):
        hits = []
        for step in range(-26, 27):
            x = centre_x+step*.5
            hit = unreal.SystemLibrary.line_trace_single(
                world, unreal.Vector(x*100-origin.x, y*100-origin.y, 500),
                unreal.Vector(x*100-origin.x, y*100-origin.y, -1500),
                unreal.TraceTypeQuery.TRACE_TYPE_QUERY1, False, ignore,
                unreal.DrawDebugTrace.NONE, True)
            values = hit.to_tuple() if hit else None
            blocked = values and values[0]
            hits.append({'ue_x_m': round(x, 3),
                         'actor': values[9].get_name() if blocked and values[9] else None,
                         'surface_z_m': round((values[5].z+origin.z)/100, 3) if blocked else None})
        rows.append({'column': column, 'ue_y_m': y, 'hits': hits})
output = ROOT/'runs/ue_scene/lane_surface_probe.json'
output.write_text(json.dumps({'note': '1.10 m asphalt versus 1.25 m sidewalk; higher hits may be overhead objects',
                             'rows': rows}, indent=2)+'\n')
unreal.log('Lane surfaces written to '+str(output))
