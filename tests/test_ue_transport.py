import json
import math
from pathlib import Path
import pytest
from logistics_park.routing.shortest_path import shortest_path
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.simulation.lightweight.ue_transport_cli import simulate

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def model():return TransportMap(ROOT/'config/topology/industrialarea_ue_transport.json')


def test_all_site_pairs_have_legal_roundtrips_and_preserve_base_lane_rules(model):
    for a in 'ABCD':
        for b in 'ABCD':
            if a==b:continue
            plans=[model.plan(a,[b,a],algorithm) for algorithm in ('astar','dijkstra')]
            assert plans[0].length==pytest.approx(plans[1].length)
            assert len(plans[0].checkpoints)==4
            assert any(k=='uturn_'+b for k in plans[0].edges)
            assert any(k=='uturn_'+a for k in plans[0].edges)
            model.graph.validate_route(plans[0].edges)
    base=json.loads((ROOT/'config/topology/industrialarea_ue.json').read_text())
    for e in base['edges']:
        if e['edge_id'].startswith('turn_'):
            row=model.edges[e['edge_id']]
            assert row['incoming_lane']==e['incoming_lane'] and row['maneuver']==e['maneuver']
    assert 'turn_j_2_0_S_E_right_single' not in model.edges
    assert 'turn_j_2_0_N_E_left_single' not in model.edges


def test_uturn_radius_and_terminal_straight_gate(model):
    maximum=math.tan(math.radians(40))/2.3285
    for label in 'ABCD':
        e=model.edges['uturn_'+label];p=e['centerline']
        for a,b,c in zip(p,p[1:],p[2:]):
            ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
            if min(ab,bc,ac)<1e-8:continue
            k=2*abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/(ab*bc*ac)
            assert k<maximum+1e-6
        if label in 'BD':
            assert e['straight_until_m']>3
            assert p[0][0]==pytest.approx(34.9817)
            assert p[1][0]>p[0][0]


def test_roundtrip_projection_stays_on_current_traversal(model):
    p=model.plan('B',['A','B','A','B'])
    repeated=[(i,j) for i,e in enumerate(p.edges) for j,f in enumerate(p.edges) if j>i and e==f]
    i,j=next((i,j) for i,j in repeated if p.edge_ends[i]-p.edge_starts[i]>10)
    s=p.edge_starts[j]+5;error,projected=p.project(p.point(s),s)
    assert error<1e-8 and projected==pytest.approx(s)


def test_split_roads_keep_physical_capacity_and_egress_clears_zones(model):
    for e in model.edges.values():assert e['capacity_alias'] in model.raw['physical_capacities']
    for site in 'AC':
        p=model.graph.nodes[model.sites[site]['egress_node']].position
        for spec in model.zones.values():
            x0,y0,x1,y1=spec['bounds']
            assert not (x0-1.3<=p[0]<=x1+1.3 and y0-5.0<=p[1]<=y1+5.0)


def test_lightweight_single_completes_continuous_roundtrip(model):
    report,frames,plans=simulate(model,1,1,1000.)
    assert report['status']=='PASS',report['failure']
    assert report['completed_tasks']==2 and report['collected_vehicles']==1
    assert report['collision_count']==report['unauthorized_resource_entry_count']==0
    events=report['task_events']
    assert len([e for e in events if e['event']=='TURNAROUND_COMPLETE'])==2
    assert frames and report['ue_physics_validated'] is False


def test_shared_corridor_is_attached_to_both_directions(model):
    forward=model.plan('A',['B'])
    reverse=model.plan('B',['A'])
    assert forward.shared_visits and reverse.shared_visits
    assert forward.shared_visits[0].channel_id == 'shared_corridor_j1'
    assert reverse.shared_visits[0].channel_id == 'shared_corridor_j1'


def test_eight_trucks_share_terminal_turns_and_complete_tasks(model):
    report,_,_=simulate(model,8,1,1500.,outlet_block_until={'j_3_2':300.})
    assert report['status']=='PASS',report['failure']
    assert report['completed_tasks']==16 and report['collected_vehicles']==8
    # Faster travel can finish a truck before the last one starts; require
    # substantial overlap while checking all eight still complete their work.
    assert report['maximum_moving_vehicles']>=7 and report['maximum_junction_queue']>=1
    assert report['collision_count']==report['unauthorized_resource_entry_count']==report['road_capacity_violation_steps']==0
    assert any('road_j_1_0_j_2_0' in alias for alias in report['road_aliases_used'])
    assert any('road_j_2_1_j_1_1' in alias for alias in report['road_aliases_used'])
    assert any(e['event']=='OUTLET_REOPENED' for e in report['task_events'])
    order=[e['vehicle_id'] for e in report['traffic_events'] if e['event']=='ENTERED' and e['intersection_id']=='j_3_2']
    assert order[:2]==['Car_0_1','Car_0_2']
