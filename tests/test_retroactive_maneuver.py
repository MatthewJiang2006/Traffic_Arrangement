"""A real stopped diversion must preserve past maneuver permissions as well as points."""
import json
from pathlib import Path
import pytest
from logistics_park.common.messages import VehicleState
from logistics_park.runtime.transport_control import TransportController
from logistics_park.runtime.transport_routes import TransportMap,RouteGeometry
from logistics_park.scheduling.conflict_timing import maneuver_groups

ROOT=Path(__file__).resolve().parents[1]


def test_recorded_q30_diversion_cannot_reopen_a_cleared_junction():
    row=json.loads((ROOT/'tests/fixtures/ue_20261007_retroactive_maneuver.json').read_text())
    model=TransportMap(ROOT/'config/topology/industrialarea_ue_internal_waiting_transport.json')
    old=RouteGeometry(model,row['original']['route_edges'],[],None,['W5-unloading'])
    new=RouteGeometry(model,row['diverted']['route_edges'],[],None,['Q30'])
    controller=TransportController(model,{'truck':old},local_traffic=True,retain_finished_vehicles=True)
    controller.activate('truck',0)
    execution=controller.executions['truck'];execution.progress=row['original']['progress_m']
    state=VehicleState('truck',**row['state'])
    # The old junction is legally cleared. Identical geometry for the full
    # retained prefix still allows the new access to merge with that junction.
    assert not any(entry+.2<execution.progress<end+.2 for _,_,entry,end in maneuver_groups(old))
    assert any(entry+.2<execution.progress<end+.2 for _,_,entry,end in maneuver_groups(new))
    for i in range(int((execution.progress+6)/.25)+1):
        assert old.point(i*.25)==pytest.approx(new.point(i*.25),abs=.001)
    before=dict(controller.capacity.committed)
    with pytest.raises(ValueError,match='retroactively'):
        controller.reroute_stopped('truck',new,1,state=state)
    assert execution.plan is old and controller.plans['truck'] is old
    assert controller.capacity.committed==before
    assert controller.local_unauthorized_count==0
