import pytest
from logistics_park.runtime.drivable_audit import audit_frames
from logistics_park.runtime.safety_timing import validate_native_policy
from test_drivable_surface import surface,rectangle


def frame(x=0.,names=('truck',)):
    return dict(positions={n:[x,0.,0.,0.] for n in names},rear_z_m={n:1.29 for n in names})


def test_full_native_audit_checks_all_bodies_and_inter_frame_sweeps():
    s=surface(rectangle(-30,-4,-.2,4),rectangle(.2,-4,30,4))
    report=audit_frames([frame(-6),frame(6)],s)
    assert report['body_samples']==2 and report['body_sweeps']==1
    assert report['status']=='FAIL'
    assert report['flags'][0]['kind']=='sweep'


def test_full_native_audit_rejects_empty_missing_height_and_disappearing_vehicle():
    s=surface(rectangle(-30,-4,30,4))
    for rows in ([],[dict(positions={},rear_z_m={})],[dict(positions={'truck':[0,0,0]},rear_z_m={})],
                 [frame(names=('a','b')),frame(names=('a',))]):
        with pytest.raises(RuntimeError):audit_frames(rows,s)


def test_full_native_audit_counts_every_fleet_member():
    s=surface(rectangle(-30,-4,30,4));names=tuple(str(i) for i in range(16))
    report=audit_frames([frame(names=names),frame(.1,names)],s)
    assert report['status']=='PASS' and report['body_samples']==32 and report['body_sweeps']==16


def test_native_policy_rejects_configuration_drift_and_time_acceleration():
    policy=dict(require_drivable_surface=True,native_control_timeout_s=.5,
                maximum_native_frame_s=.1,reaction_budget_s=1.2,maximum_source_state_age_s=.5)
    validate_native_policy(policy,1)
    for key in policy:
        changed=dict(policy);changed[key]=False if isinstance(policy[key],bool) else policy[key]+.1
        with pytest.raises(ValueError):validate_native_policy(changed,1)
    with pytest.raises(ValueError):validate_native_policy(policy,2)
