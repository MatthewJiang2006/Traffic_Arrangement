"""Protect continuous road merges and verified reverse access publication."""
import hashlib
import json
import math
from pathlib import Path

from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.transport_control import envelope
from logistics_park.common.messages import VehicleState
from logistics_park.safety.collision import overlaps
from logistics_park.runtime.ue_transport import configured_plans

ROOT=Path(__file__).resolve().parents[1]


def test_smooth_connections_obey_curvature_and_endpoint_direction():
    data=json.loads((ROOT/'config/planning/ue_access_connection_acceptance.json').read_text())
    assert len(data['connections']) == 9 and len(data['verified_reverse_sites']) == 4
    for c in data['connections'].values():
        path=c['centerline']
        assert c['maximum_curvature_m_inv'] < math.tan(math.radians(40))/2.3285
        road=path[0] if c['direction']=='entry' else path[-1]
        assert math.dist(road,c['road_rear_xy_m']) < 1e-8
        a,b=path[:2] if c['direction']=='entry' else path[-2:]
        motion=math.atan2(b[1]-a[1],b[0]-a[0])
        body=motion+(math.pi if c['reverse'] else 0)
        assert abs((body+math.pi/2+math.pi)%(2*math.pi)-math.pi) < .02


def test_published_connections_use_actual_curves_and_preserved_map():
    approved=json.loads((ROOT/'config/planning/ue_access_connection_acceptance.json').read_text())
    graph=json.loads((ROOT/'config/topology/industrialarea_ue_service_network.json').read_text())
    assert graph['map_sha256'] == approved['map_sha256']
    assert approved['static_actor_preservation']['changed'] == []
    assert set(approved['static_actor_preservation']['removed']) == {'SM_Block_Wall_57','SM_Block_Wall_68'}
    assert hashlib.sha256((ROOT/'config/planning/ue_access_connection_candidates.json').read_bytes()).hexdigest() == approved['source_candidates_sha256']
    assert sum(p.get('ue_boundary_validated',False) for p in graph['road_ports']) == 9
    curves=[e for e in graph['edges'] if e.get('phase','').endswith('attachment') and e.get('physical_evidence')]
    assert len(curves)==9 and all(len(e['centerline'])>10 for e in curves)
    assert not any(s['access_validation_required'] for s in graph['access_sequences'])


def test_batch_routes_preserve_gear_stops_and_spatial_separation():
    for number in (1,2,3):
        scenario=json.loads((ROOT/f'config/runtime/industrialarea_ue_access_batch_{number}.json').read_text())
        model=TransportMap(ROOT/scenario['map']);plans=configured_plans(model,scenario)
        assert 'TrafficStationQ3CurbRamp' in scenario['verified_ground_actors']
        sweep={}
        for name,plan in plans.items():
            stations=[min(plan.length,k*.5) for k in range(math.ceil(plan.length/.5)+1)]
            sweep[name]=[envelope(VehicleState(name,*plan.point(s),plan.heading(s))) for s in stations]
        names=list(sweep)
        for i,a in enumerate(names):
            for b in names[i+1:]:
                assert not any(overlaps(x,y,.25) for x in sweep[a] for y in sweep[b])
        if number==3:
            plan=next(iter(plans.values()))
            changes=[c for c in plan.checkpoints if c['event']=='GEAR_CHANGE']
            assert len(changes)==1 and changes[0]['direction']==-1
            assert plan.speed_cap(changes[0]['s'])==0
