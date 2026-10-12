import json,math
from pathlib import Path
from types import SimpleNamespace

import pytest

from logistics_park.runtime.node_pair_validation import can_stage, clear_collected, clear_aborted
from logistics_park.runtime.transport_routes import TransportMap, RouteGeometry
from logistics_park.runtime.transport_control import TransportController
from logistics_park.scheduling.shared_resource import SharedResourceRequest

ROOT=Path(__file__).resolve().parents[1]


def test_all_ordered_service_pairs_have_continuous_directed_routes():
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').read_text())
    model=TransportMap(ROOT/spec['map'])
    nodes=set(spec['node_ids'])
    assert len(nodes)==12 and all(n.startswith('W') for n in nodes)
    assert {(j['origin'],j['destination']) for j in spec['jobs']}=={(a,b) for a in nodes for b in nodes if a!=b}
    assert len(spec['jobs'])==132
    for job in spec['jobs']:
        p=RouteGeometry(model,job['edges'],job['checkpoints'],job['origin'],[job['destination']])
        assert p.length>0
        assert all(model.edges[a]['to_node']==model.edges[b]['from_node'] for a,b in zip(job['edges'],job['edges'][1:]))
        for cp in p.checkpoints:
            if cp['event']!='GEAR_CHANGE':continue
            before=p.heading(max(0,cp['s']-.02));after=p.heading(min(p.length,cp['s']+.02))
            assert abs((after-before+math.pi)%(2*math.pi)-math.pi)<.35


def test_collection_requires_completed_arrival_and_physical_removal():
    c=SimpleNamespace(collect_at_destination=True,executions={'truck':SimpleNamespace(finished=False)})
    with pytest.raises(ValueError):clear_collected(c,'truck',1,physically_removed=True)
    c.executions['truck'].finished=True
    with pytest.raises(ValueError):clear_collected(c,'truck',1,physically_removed=False)
    c.collect_at_destination=False
    with pytest.raises(ValueError):clear_collected(c,'truck',1,physically_removed=True)


def test_collection_releases_only_removed_trucks_reservations():
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').read_text())
    model=TransportMap(ROOT/spec['map']);job=spec['jobs'][0]
    plan=RouteGeometry(model,job['edges'],job['checkpoints'],job['origin'],[job['destination']])
    c=TransportController(model,{'a':plan,'b':plan},shared_resources=True,collect_at_destination=True)
    c.activate('a',0);c.activate('b',0);c.executions['a'].finished=True
    channel=next(iter(model.shared_resources))
    c.shared_scheduler.request(channel,'a',0);c.shared_scheduler.request(channel,'b',0)
    # A repeated itinerary may already have a later visit queued under the
    # same vehicle ID. Removal must cancel it before releasing its old lease.
    c.shared_scheduler.queues[channel]['a']=SharedResourceRequest(channel,'a',0,0)
    clear_collected(c,'a',1,physically_removed=True)
    assert 'a' not in c.executions and 'b' in c.executions
    assert c.shared_scheduler.allows(channel,'b')


def test_staging_waits_for_remaining_conflicting_route_to_clear():
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').read_text())
    model=TransportMap(ROOT/spec['map']);job=spec['jobs'][0]
    plan=RouteGeometry(model,job['edges'],job['checkpoints'],job['origin'],[job['destination']])
    c=TransportController(model,{'a':plan},shared_resources=True,collect_at_destination=True)
    c.activate('a',0)
    assert not can_stage(c,plan)
    c.executions['a'].shared_cleared=set(range(len(plan.shared_visits)))
    assert can_stage(c,plan)


def test_failed_test_is_cleared_without_fabricating_arrival():
    spec=json.loads((ROOT/'config/runtime/industrialarea_ue_node_pair_tests.json').read_text())
    model=TransportMap(ROOT/spec['map']);job=spec['jobs'][0]
    p=RouteGeometry(model,job['edges'],job['checkpoints'],job['origin'],[job['destination']])
    c=TransportController(model,{'a':p},shared_resources=True,collect_at_destination=True)
    c.activate('a',0)
    assert not c.executions['a'].finished
    with pytest.raises(ValueError):clear_aborted(c,'a',1,physically_removed=False)
    channel=next(iter(model.shared_resources));c.shared_scheduler.request(channel,'a',0)
    clear_aborted(c,'a',1,physically_removed=True)
    assert 'a' not in c.executions and not c.shared_scheduler.owners
    assert any(e['event']=='TEST_PAIR_ABORTED' for e in c.events)
    assert not any(e['event'] in ('ARRIVED','TASK_COMPLETE','COLLECTED') for e in c.events)
