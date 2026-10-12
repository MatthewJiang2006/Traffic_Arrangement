import pytest
from logistics_park.runtime.diagnostic_precheck import prefix_precheck_ready


def ready():
    return dict(status='DIAGNOSTIC_READY',failure=None,diagnostic_preflight_s=120,
        elapsed_sim_s=120,vehicle_count=16,collision_count=0,unauthorized_shared_entry_count=0,
        local_unauthorized_entry_count=0,road_capacity_violation_steps=0,completed_tasks=0)


def test_bounded_diagnostic_is_explicit_and_can_have_incomplete_tasks():
    assert prefix_precheck_ready(ready(),16,120)
    assert not prefix_precheck_ready(ready(),24,120)
    assert not prefix_precheck_ready(ready(),16,60)


@pytest.mark.parametrize('change',[{'status':'FAIL'},{'failure':'collision'},
    {'collision_count':1},{'unauthorized_shared_entry_count':1},{'local_unauthorized_entry_count':1},
    {'road_capacity_violation_steps':1},{'elapsed_sim_s':119},{'elapsed_sim_s':float('inf')},
    {'diagnostic_preflight_s':30},{'vehicle_count':8}])
def test_unsafe_incomplete_or_wrong_fleet_precheck_is_rejected(change):
    report=ready();report.update(change)
    assert not prefix_precheck_ready(report,16,120)
