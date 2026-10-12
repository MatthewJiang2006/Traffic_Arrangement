from dataclasses import replace
from pathlib import Path
import pytest

from logistics_park.common.messages import VehicleState
from logistics_park.map.map_loader import load_map
from logistics_park.scheduling.fcfs import EntranceRequest,FCFSScheduler
from logistics_park.scheduling.road_capacity import RoadCapacity

ROOT=Path(__file__).resolve().parents[1]


@pytest.fixture
def capacity():
    return RoadCapacity(load_map(ROOT/'config/topology/fcfs_four_way.json'))


def request(vehicle,at=0,lane='approach_W',connector='cross_W_E',exit_edge='exit_E',s=.4,junction='cross'):
    return EntranceRequest(vehicle,junction,lane,connector,exit_edge,at,s)


def test_commitment_conversion_and_idempotency(capacity):
    assert capacity.reserve('v','exit_E')
    assert capacity.reserve('v','exit_E')
    assert capacity.road_state('exit_E').committed_vehicle_count==1
    state=VehicleState('v',1.5,.9,0,edge_id='exit_E',s=.15)
    capacity.update({'v':state})
    road=capacity.road_state('exit_E')
    assert road.current_vehicle_count==1 and road.committed_vehicle_count==0
    assert capacity.reserve('v','exit_E')  # no double count of the same vehicle
    assert capacity.max_effective['exit_E']==1
    assert capacity.reserve('second','exit_E') and capacity.reserve('third','exit_E')
    assert not capacity.reserve('fourth','exit_E')
    assert capacity.violations==0
    capacity.release_commitment('third','exit_E')
    assert capacity.reserve('fourth','exit_E')
    with pytest.raises(ValueError):
        capacity.reserve('v','missing')


def test_zero_capacity_rejects_all_new_entrants(capacity):
    capacity.capacities['exit_E']=0
    assert not capacity.reserve('v','exit_E')
    assert capacity.road_state('exit_E').is_saturated


def test_strict_fcfs_does_not_bypass_blocked_head(capacity):
    states={f'parked{i}':VehicleState(f'parked{i}',1.7,.9,0,edge_id='exit_E',s=.2+i*.2) for i in range(3)}
    capacity.update(states)
    scheduler=FCFSScheduler(('cross',),capacity)
    scheduler.request(request('first',0))
    scheduler.request(request('second',.1,'approach_N','cross_N_S','exit_S'))
    scheduler.advance(1)
    assert scheduler.owners=={} and scheduler.capacity_blocked_attempts==1
    assert capacity.road_state('exit_S').committed_vehicle_count==0
    capacity.update({})
    scheduler.advance(2)
    assert scheduler.owners['cross'].request.vehicle_id=='first'
    assert capacity.road_state('exit_E').committed_vehicle_count==1
    scheduler.advance(3)
    assert len([event for event in scheduler.events if event['event']=='GRANTED'])==1


def test_lane_fifo_pruning_and_request_timestamp_preserved(capacity):
    scheduler=FCFSScheduler(('cross',),capacity)
    scheduler.request(request('rear',0,s=.1))
    scheduler.request(request('front',.1,s=.5))
    scheduler.request(request('other',.05,'approach_N','cross_N_S','exit_S',s=.4))
    scheduler.request(request('rear',.3,s=.2))
    assert scheduler.queues['cross']['rear'].requested_at==0
    scheduler.advance(1)
    assert scheduler.owners['cross'].request.vehicle_id=='other'
    scheduler.mark_entered('cross','other',1.1)
    scheduler.complete('cross','other',2)
    scheduler.advance(2)
    assert scheduler.owners['cross'].request.vehicle_id=='front'
    scheduler.mark_entered('cross','front',2.1)
    scheduler.complete('cross','front',3)
    scheduler.advance(3)
    assert scheduler.owners['cross'].request.vehicle_id=='rear'


def test_independent_junctions_can_own_different_vehicles(capacity):
    scheduler=FCFSScheduler(('cross','fork'),capacity)
    scheduler.request(request('v1'))
    scheduler.request(request('v2',lane='exit_E',connector='fork_0',exit_edge='parking_E_0',junction='fork'))
    scheduler.advance(1)
    assert len(scheduler.owners)==scheduler.max_owners==2
    assert scheduler.allows('v1','cross','cross_W_E')
    assert not scheduler.allows('v2','cross','cross_W_E')
    with pytest.raises(ValueError):
        scheduler.complete('cross','v1',1)  # never entered
    with pytest.raises(ValueError):
        scheduler.mark_entered('cross','v2',1)
    scheduler.mark_entered('cross','v1',1.5)
    scheduler.complete('cross','v1',2)
    assert 'cross' not in scheduler.owners and 'fork' in scheduler.owners
    # Completion does not discard the downstream capacity promise prematurely.
    assert capacity.road_state('exit_E').committed_vehicle_count==1


def test_entry_gap_guard_and_invalid_request(capacity):
    scheduler=FCFSScheduler(('cross',),capacity)
    scheduler.request(request('v'))
    scheduler.advance(0,lambda edge,vehicle:False)
    assert scheduler.owners=={} and scheduler.entry_blocked_attempts==1
    assert capacity.road_state('exit_E').committed_vehicle_count==0
    with pytest.raises(ValueError):
        scheduler.request(request('v',connector='cross_W_N',exit_edge='exit_N'))
    with pytest.raises(ValueError):
        scheduler.request(request('missing',junction='unknown'))
    with pytest.raises(ValueError):
        scheduler.request(request('v2',at=float('nan')))
