import copy
import json
from pathlib import Path

from logistics_park.runtime.route_coverage import coverage_result, coverage_sets, select_scan

ROOT=Path(__file__).resolve().parents[1]


def fixture():
    scenario=json.loads((ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').read_text())
    topology=json.loads((ROOT/scenario['map']).read_text())
    return topology,scenario


def test_scan_covers_all_roads_transitions_endpoints_and_gears():
    topology,scenario=fixture()
    plan=select_scan(topology,scenario)
    tokens=plan['requirements_by_pair']
    assert set().union(*(set(tokens[k]) for k in plan['selected_pair_ids']))==set().union(*(set(v) for v in tokens.values()))
    assert plan['source_route_count']==132 and plan['selected_route_count']<40
    assert plan['distance_reduction_fraction']>.8
    assert all(plan['requirement_counts'][kind]>0 for kind in
               ('EDGE','JOIN','CHAIN3','DEPARTURE','ARRIVAL','GEAR_CHANGE'))
    assert not plan['ue_physics_validated']


def test_geometry_clones_merge_but_reverse_speed_delay_and_resources_do_not():
    topology,scenario=fixture()
    edge=copy.deepcopy(topology['edges'][0])
    edge['edge_id']='base'
    changes=[{},dict(reverse=True),dict(speed_limit=.5),dict(straight_until_m=1),dict(required_resources=['exclusive'])]
    edges=[edge]
    for i,change in enumerate(changes):
        clone=copy.deepcopy(edge);clone.update(edge_id='copy'+str(i),**change);edges.append(clone)
    jobs=[dict(pair_id=e['edge_id'],origin='origin',destination='dest',edges=[e['edge_id']],
               checkpoints=[dict(event='ARRIVAL',after_edge_count=1)]) for e in edges]
    requirements=coverage_sets(dict(edges=edges),jobs)
    assert requirements['base']==requirements['copy0']
    for i in range(1,5):assert requirements['base']!=requirements['copy'+str(i)]


def physical_passes(plan):
    return [dict(pair_id=key,status='PASS',ue_physics_validated=True,
                 body_obstacle_contacts=0,vehicle_collisions=0) for key in plan['selected_pair_ids']]


def test_complete_component_evidence_is_not_132_end_to_end_evidence():
    plan=select_scan(*fixture())
    result=coverage_result(plan,physical_passes(plan))
    assert result['coverage_status']=='COMPLETE'
    assert len(result['components_by_pair'])==132
    assert set(result['components_by_pair'].values())=={'COMPONENTS_COVERED'}
    assert not result['end_to_end_132_physically_validated']


def test_failed_route_does_not_cover_untraversed_roads():
    plan=select_scan(*fixture())
    rows=physical_passes(plan)
    rows[0]['status']='FAIL'
    result=coverage_result(plan,rows)
    assert result['coverage_status']=='INCOMPLETE' and result['missing_requirements']


def test_lightweight_or_contacting_results_cannot_publish_physical_coverage():
    plan=select_scan(*fixture())
    rows=physical_passes(plan)
    for row in rows:row['ue_physics_validated']=False
    assert coverage_result(plan,rows)['covered_requirement_count']==0
    rows=physical_passes(plan)
    for row in rows:row['body_obstacle_contacts']=1
    assert coverage_result(plan,rows)['covered_requirement_count']==0
    rows=physical_passes(plan)
    for row in rows:row['vehicle_collisions']=1
    assert coverage_result(plan,rows)['covered_requirement_count']==0
