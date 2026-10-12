import pytest

from logistics_park.scheduling.fleet_lifecycle import FleetPhase, FleetRegistry, GateDirection, GatePermit


def enter(registry, vehicle_id='new-truck'):
    row=registry.register_vehicle(vehicle_id,0)
    registry.request_entry(vehicle_id,row.visit_id,'entry',1)
    permit=GatePermit('entry-lease',vehicle_id,row.visit_id,'entry',GateDirection.ENTRY,1,5)
    registry.authorize_entry(permit,2)
    return registry.confirm_entry(vehicle_id,row.visit_id,3,fully_inside=True)


def exit_permit(row):
    return GatePermit('exit-lease',row.vehicle_id,row.visit_id,'exit',GateDirection.EXIT,5,20)


def test_new_vehicle_is_dispatchable_only_after_physical_entry():
    r=FleetRegistry();v=r.register_vehicle('new',0)
    assert not r.available_vehicle_ids and not r.park_vehicle_ids
    with pytest.raises(ValueError):r.assign_task('new',v.visit_id,'task',0)
    r.request_entry('new',v.visit_id,'entry',1)
    r.authorize_entry(GatePermit('lease','new',v.visit_id,'entry',GateDirection.ENTRY,1,4),2)
    assert r.park_vehicle_ids==('new',) and not r.available_vehicle_ids
    with pytest.raises(ValueError):r.confirm_entry('new',v.visit_id,3,fully_inside=False)
    r.confirm_entry('new',v.visit_id,3,fully_inside=True)
    assert r.available_vehicle_ids==('new',)


def test_exit_request_drains_current_task_and_blocks_new_assignments():
    r=FleetRegistry();v=enter(r)
    r.assign_task(v.vehicle_id,v.visit_id,'existing-task',4)
    assert r.request_exit(v.vehicle_id,v.visit_id,'exit',5).phase==FleetPhase.DRAINING
    with pytest.raises(ValueError):r.assign_task(v.vehicle_id,v.visit_id,'new-task',5)
    with pytest.raises(ValueError):r.authorize_exit(exit_permit(v),6)
    with pytest.raises(ValueError):r.finish_task(v.vehicle_id,v.visit_id,'wrong-task',6)
    assert r.finish_task(v.vehicle_id,v.visit_id,'existing-task',6).phase==FleetPhase.EXIT_REQUESTED
    assert not r.available_vehicle_ids
    assert r.authorize_exit(exit_permit(v),7).phase==FleetPhase.EXITING


def test_departure_requires_body_clearance_and_fresh_resource_cleanup():
    r=FleetRegistry();v=enter(r)
    r.set_owned_resources(v.vehicle_id,v.visit_id,[],4)
    r.request_exit(v.vehicle_id,v.visit_id,'exit',5)
    r.authorize_exit(exit_permit(v),6)
    with pytest.raises(ValueError):r.confirm_exit(v.vehicle_id,v.visit_id,7,fully_outside=True)
    r.set_owned_resources(v.vehicle_id,v.visit_id,['road:1','gate:exit','waiting:Q1'],7)
    with pytest.raises(ValueError):r.confirm_exit(v.vehicle_id,v.visit_id,7,fully_outside=True)
    r.set_owned_resources(v.vehicle_id,v.visit_id,[],8)
    with pytest.raises(ValueError):r.confirm_exit(v.vehicle_id,v.visit_id,8,fully_outside=False)
    with pytest.raises(ValueError):r.confirm_exit(v.vehicle_id,v.visit_id,9,fully_outside=True)
    r.confirm_exit(v.vehicle_id,v.visit_id,8,fully_outside=True)
    assert not r.park_vehicle_ids and not r.available_vehicle_ids


def test_reentry_rejects_old_session_events_and_preserves_history():
    r=FleetRegistry();old=enter(r)
    r.request_exit(old.vehicle_id,old.visit_id,'exit',5);r.authorize_exit(exit_permit(old),6)
    r.set_owned_resources(old.vehicle_id,old.visit_id,[],7)
    r.confirm_exit(old.vehicle_id,old.visit_id,7,fully_outside=True)
    new=r.register_vehicle(old.vehicle_id,8)
    assert new.visit_id!=old.visit_id and r.visits[old.visit_id].phase==FleetPhase.DEPARTED
    with pytest.raises(ValueError):r.request_entry(old.vehicle_id,old.visit_id,'entry',9)
    with pytest.raises(ValueError):r.set_owned_resources(old.vehicle_id,old.visit_id,[],9)
    assert r.vehicles[old.vehicle_id].phase==FleetPhase.OUTSIDE


@pytest.mark.parametrize('gate,direction,valid_after,valid_until',[
    ('wrong',GateDirection.ENTRY,1,5),('entry',GateDirection.EXIT,1,5),
    ('entry',GateDirection.ENTRY,3,5),('entry',GateDirection.ENTRY,0,2)])
def test_gate_permits_must_match_direction_gate_and_time(gate,direction,valid_after,valid_until):
    r=FleetRegistry();v=r.register_vehicle('new',0);r.request_entry('new',v.visit_id,'entry',1)
    with pytest.raises(ValueError):r.authorize_entry(GatePermit('lease','new',v.visit_id,gate,direction,valid_after,valid_until),2)
    assert r.vehicles['new'].phase==FleetPhase.ENTRY_REQUESTED


def test_duplicate_registration_tasks_and_out_of_order_events_are_rejected():
    r=FleetRegistry();a=enter(r,'a');b=enter(r,'b')
    with pytest.raises(ValueError):r.register_vehicle('a',4)
    r.assign_task('a',a.visit_id,'task',4)
    with pytest.raises(ValueError):r.assign_task('b',b.visit_id,'task',4)
    with pytest.raises(ValueError):r.request_exit('a',a.visit_id,'exit',3)
    assert r.vehicles['b'].phase==FleetPhase.IDLE


def test_existing_park_fleet_can_be_adopted_without_a_fictional_entry():
    r=FleetRegistry()
    with pytest.raises(ValueError):r.adopt_present_vehicle('present',0,fully_inside=False,resources=[])
    with pytest.raises(ValueError):r.adopt_present_vehicle('present',0,fully_inside=True,resources='road:1')
    row=r.adopt_present_vehicle('present',0,fully_inside=True,resources=['waiting:Q1'])
    assert r.available_vehicle_ids==('present',) and row.owned_resources==frozenset({'waiting:Q1'})
    assert not any(e['event']=='VEHICLE_ENTERED' for e in r.events)
