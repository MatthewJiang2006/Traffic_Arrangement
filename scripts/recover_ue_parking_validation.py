#!/usr/bin/env python3
"""Recheck restoration after a completed parking scan, preserving its raw FAIL."""
import argparse,copy,hashlib,json,math,sys,time
from pathlib import Path
import airsim
from ue_dynamic_crossing_check import absolute_ue
from ue_trailer_driving_check import ue_script
from validate_ue_loading_access import map_fingerprints
ROOT=Path(__file__).resolve().parents[1]


def run(folder):
    raw_path=folder/'waiting_acceptance.json';raw=json.loads(raw_path.read_text())
    if (raw['status']!='FAIL' or raw['completed_pairs']!=13 or raw['failed_pairs']!=0
            or raw['body_obstacle_contacts'] or raw['vehicle_collisions'] or raw.get('failure')):
        raise ValueError('Recovery applies only to a complete collision-free scan with failed restoration')
    if raw['map_sha256']!=map_fingerprints():raise ValueError('Scene differs from the tested map')
    scene=json.loads((ROOT/'runs/ue_scene/scene.json').read_text())
    origin=[v/100 for v in scene['world_origin_cm']]
    expected={v['name']:[(p+o)/100 for p,o in zip(v['location_from_origin_cm'],scene['world_origin_cm'])] for v in scene['pie_vehicles']}
    c=airsim.CarClient(timeout_value=10);names=c.listVehicles()
    if set(names)!=set(expected):raise ValueError('Original 24-truck fleet required')
    errors={n:math.dist(absolute_ue(c,n,origin)[0][:2],expected[n][:2]) for n in names}
    if max(errors.values())>.1:raise ValueError('Restarted fleet does not match original world poses: '+str(errors))
    apis={n:c.isApiControlEnabled(vehicle_name=n) for n in names}
    if any(apis.values()):raise ValueError('Original scan used keyboard API states; restarted fixture must be disabled')
    fixture_dir=ROOT/'runs/ue_node_pairs'
    def fixture(action,vehicles=None):
        (fixture_dir/'fleet_request.json').write_text(json.dumps(dict(action=action,vehicles=vehicles or names,time_scale=1,output_dir=str(folder))))
        return ue_script(c,'node_pair_fleet_fixture.py',folder/'fleet_status.json')
    # Reproduce one active-car local-frame reset without re-running any road route.
    fixture('INIT')
    try:
        fixture('ACTIVATE',['Car_0_0']);c.enableApiControl(True,vehicle_name='Car_0_0')
        c.setCarControls(airsim.CarControls(brake=1,handbrake=True),vehicle_name='Car_0_0')
        local=c.simGetVehiclePose(vehicle_name='Car_0_0');actual=c.simGetObjectPose('Car_0_0')
        target=[-110-origin[0],26-origin[1],origin[2]-.894]
        new=airsim.Pose(airsim.Vector3r(local.position.x_val+target[0]-actual.position.x_val,local.position.y_val+target[1]-actual.position.y_val,local.position.z_val+target[2]-actual.position.z_val),airsim.to_quaternion(0,0,math.pi/2))
        c.simPause(True);c.simSetVehiclePose(new,True,vehicle_name='Car_0_0');c.simPause(False);time.sleep(.3)
    finally:
        c.reset();c.simPause(True)
        try:
            fixture('RESTORE')
            # The fix: trust the saved UE world transform; no stale local-pose replay.
            stop=ROOT/'runs/ue_scene/test_sites'
            (stop/'motion_stop_request.json').write_text(json.dumps(dict(vehicles=names)))
            ue_script(c,'stop_test_vehicle_motion.py',stop/'motion_stop.json')
        finally:c.simPause(False)
        time.sleep(.5)
        for n,a in apis.items():c.enableApiControl(a,vehicle_name=n)
    observed=[]
    for _ in range(3):
        observed.append(dict(maximum_world_xy_error_m=max(math.dist(absolute_ue(c,n,origin)[0][:2],expected[n][:2]) for n in names),maximum_speed_m_s=max(abs(c.getCarState(vehicle_name=n).speed) for n in names),api_restored=all(c.isApiControlEnabled(vehicle_name=n)==apis[n] for n in names)))
        time.sleep(.25)
    passed=all(r['maximum_world_xy_error_m']<.1 and r['maximum_speed_m_s']<.1 and r['api_restored'] for r in observed)
    record=dict(status='PASS' if passed else 'FAIL',observations=observed,vehicle_count=len(names),original_scan_sha256=hashlib.sha256(raw_path.read_bytes()).hexdigest(),saved_maps_unchanged=map_fingerprints()==raw['map_sha256'],camera_changed=False,scope='Post-scan restarted fleet + active-car reset/restore regression; no route retest')
    path=folder/'restoration_recovery.json';path.write_text(json.dumps(record,indent=2)+'\n')
    if not passed or not record['saved_maps_unchanged']:raise ValueError('Restoration regression failed: '+str(record))
    final=copy.deepcopy(raw)
    final.update(status='PASS',fleet_restored=True,api_restored=True,max_restored_speed_m_s=max(r['maximum_speed_m_s'] for r in observed),active_pairs={},restoration_recovery=dict(report=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),raw_scan_status='FAIL',raw_failure='post-scan fixture restoration; all 13 routes passed'))
    (folder/'waiting_recovered_acceptance.json').write_text(json.dumps(final,indent=2)+'\n')
    print('13 parking accesses PASS + restoration PASS',record['observations'])

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output-dir',type=Path,required=True);args=parser.parse_args();run(args.output_dir.resolve())
