#!/usr/bin/env python3
"""Run one acceptance exactly the way the frozen campaign does.

The campaign harness lived outside the repository; this script ports the same
four layers so ``./run.sh`` is equivalent to one campaign trial:

1. frozen hashes   - available but skipped on request;
2. prechecks       - skipped on request: no lightweight run, no regression, no
                     native gate; the workspace copy is still isolated;
3. isolation       - a fresh working copy outside the repository holds the run,
                     the output directory and every artefact;
4. UE lifecycle    - the script loads the map, starts PIE, stops the fleet,
                     runs the validator with a wall-clock guard, then restores
                     PIE, the level and the editor settings;
5. verdict         - two independent audits (pose, road envelope) are combined
                     with the acceptance report, exactly like normal_verdict.

Usage:
    ./run.sh                      # output under ../Demo_runs/ue_acceptance_<ts>
    ./run.sh --output /data/run01 # explicit external directory
"""

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import signal
import stat
import socket
import subprocess
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC_REL = 'config/runtime/industrialarea_ue_full_verified_20261007.json'
LEVEL = '/Game/IndustrialArea/Map/IndustrialArea（复件）'
UE_PROJECT_DIR = Path(os.environ.get('TRAFFIC_UE_PROJECT_DIR', '/home/aipc/Traffic_ue4')).resolve()
MAP_DIR = UE_PROJECT_DIR / 'Content/IndustrialArea/Map'
OUTPUT_ROOT = Path(os.environ.get('DEMO_OUTPUT_ROOT', ROOT.parent / 'Demo_runs'))
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', TRAFFIC_UE_PROJECT_DIR=str(UE_PROJECT_DIR))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_manifests():
    files = json.loads((ROOT / 'config/validation/frozen_files.json').read_text())
    environment = json.loads((ROOT / 'config/validation/frozen_environment.json').read_text())
    spec = json.loads((ROOT / SPEC_REL).read_text())
    return files, environment, spec


FILES, ENVIRONMENT, SPEC = load_manifests()
OUT_REL = SPEC['output']


def ue(code, timeout=35):
    result = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/ue_remote.py'), 'exec(' + repr(code) + ', {})'],
        capture_output=True, text=True, timeout=timeout, env=ENV)
    if result.returncode:
        raise RuntimeError('UE remote failed: ' + (result.stdout + result.stderr)[-2000:])
    row = json.loads(result.stdout)
    if not row.get('success'):
        raise RuntimeError(row)
    return row


def verify(root=None, extra=()):
    roots = [ROOT] if root is None else [ROOT, root]
    roots.extend(extra)
    for base in roots:
        for name, expected in FILES['files'].items():
            if name == 'run.sh':
                continue        # outer entrypoint, intentionally not frozen
            path = Path(base) / name
            if not path.is_file() or digest(path) != expected:
                raise RuntimeError('frozen source drift: %s' % path)
    for name, expected in ENVIRONMENT['native_and_geometry_sha256'].items():
        if digest(Path(name)) != expected:
            raise RuntimeError('frozen native/geometry drift: %s' % name)
    for name, expected in SPEC['map_sha256'].items():
        if digest(MAP_DIR / name) != expected:
            raise RuntimeError('frozen map drift: %s' % name)


def prepare_workspace(output):
    work = output / 'workspace'
    shutil.copytree(ROOT, work, ignore=shutil.ignore_patterns(
        '.git', 'runs', '__pycache__', '*.pyc', '.pytest_cache', '.venv'))
    # copytree keeps the source modes; the repository is write-protected, but
    # its disposable copy outside the repository must be writable.
    for path in (work, *work.rglob('*')):
        mode = path.stat().st_mode
        if path.is_dir():
            path.chmod(mode | stat.S_IWUSR | stat.S_IXUSR)
        elif path.is_file():
            path.chmod(mode | stat.S_IWUSR)
    # Bind hashes to the editor project actually under test. Preserve expected
    # digests; a copied project must contain exactly the verified dependencies.
    bound=json.loads((work / SPEC_REL).read_text())
    bindings={}
    for field in ('ue_geometry_asset_sha256','ue_native_dependency_sha256'):
        remapped={}
        for filename,expected in bound.get(field,{}).items():
            prefix='/home/aipc/Traffic_ue4/'
            target=UE_PROJECT_DIR / filename[len(prefix):] if filename.startswith(prefix) else Path(filename)
            if digest(target)!=expected:raise RuntimeError('bound UE dependency drift: '+str(target))
            remapped[str(target)]=expected;bindings[filename]=str(target)
        bound[field]=remapped
    (work / SPEC_REL).write_text(json.dumps(bound,indent=2)+'\n')
    (output / 'project_bindings.json').write_text(json.dumps(dict(project=str(UE_PROJECT_DIR),files=bindings),indent=2)+'\n')
    (work / OUT_REL).mkdir(parents=True, exist_ok=True)
    return work


def prechecks(work):
    """Regression only: the lightweight task run is intentionally skipped."""
    log_path = work / 'regression.log'
    with log_path.open('w') as log:
        result = subprocess.run(
            [sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', 'tests'],
            cwd=work, env=dict(ENV, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1',
                               PYTHONPATH=str(work / 'src') + os.pathsep + str(work / 'scripts')),
            stdout=log, stderr=subprocess.STDOUT)
    if result.returncode:
        raise RuntimeError('regression suite failed: see %s' % log_path)
    return dict(status='SKIPPED')


def native_gate(work):
    """Cover the published routes on the stored native surface export."""
    code = '''
import json,sys
sys.path.insert(0, %r)
from logistics_park.routing.service_network import ServiceNetwork
from logistics_park.runtime.transport_routes import TransportMap
from logistics_park.runtime.task_execution import ServiceRouteGeometry
from logistics_park.runtime.drivable_surface import DrivableSurface
from logistics_park.runtime.surface_preflight import verify_service_surfaces
from logistics_park.runtime.road_guard import body_polygon
from pathlib import Path
root = Path(%r)
spec = json.loads((root / %r).read_text())
surface = DrivableSurface(json.loads((root / 'config/validation/drivable_surface.json').read_text()))
geometry = ServiceRouteGeometry(ServiceNetwork(root / spec['network'], blocked_edges=spec['blocked_candidate_edges']),
                                TransportMap(root / spec['map']))
report = verify_service_surfaces(geometry, surface)
initial = {}
for vehicle, node in spec['positions'].items():
    point = geometry.stationary(node)
    pose = (*point.point(0), point.heading(0))
    initial[vehicle] = surface.covers(body_polygon(*pose), 1.29)
out = dict(status='PASS' if report['status'] == 'PASS' and all(initial.values()) else 'FAIL',
           preflight=report['status'], uncovered=[v for v, ok in initial.items() if not ok])
print(json.dumps(out))
''' % (str(work / 'src'), str(work), SPEC_REL)
    result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                            env=dict(ENV, PYTHONPATH=str(work / 'src')))
    if result.returncode:
        raise RuntimeError('native gate failed: ' + (result.stdout + result.stderr)[-1500:])
    gate = json.loads(result.stdout.strip().splitlines()[-1])
    if gate['status'] != 'PASS':
        raise RuntimeError('native gate rejected: %s' % gate)
    return gate


def save_editor_state(output):
    row = ue("import unreal,json,os\n"
             "assert os.path.realpath(unreal.Paths.project_dir())==%r\n" % str(UE_PROJECT_DIR) +
             "w=unreal.EditorLevelLibrary.get_editor_world()\n"
             "assert unreal.EditorLevelLibrary.get_game_world() is None\n"
             "c,r=unreal.EditorLevelLibrary.get_level_viewport_camera_info()\n"
             "p=unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorPerformanceSettings'))\n"
             "a=unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorLoadingSavingSettings'))\n"
             "print(json.dumps(dict(level=w.get_path_name().split('.')[0],"
             "camera=[c.x,c.y,c.z,r.pitch,r.yaw,r.roll],"
             "throttle=p.get_editor_property('bThrottleCPUWhenNotForeground'),"
             "autosave=a.get_editor_property('bAutoSaveEnable'))))\n"
             "a.set_editor_property('bAutoSaveEnable',False)")
    state = json.loads([x['output'] for x in row['output'] if str(x.get('output', '')).startswith('{')][0])
    (output / 'editor_before.json').write_text(json.dumps(state, indent=2) + '\n')
    return state


def restore_editor(state):
    ue("import unreal\n"
       "w=unreal.EditorLevelLibrary.get_editor_world()\n"
       "w.get_world_settings().set_editor_property('DefaultGameMode',None)\n"
       "unreal.EditorLevelLibrary.load_level(%r)\n"
       "unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorLoadingSavingSettings'))"
       ".set_editor_property('bAutoSaveEnable',%r)\n"
       "unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorPerformanceSettings'))"
       ".set_editor_property('bThrottleCPUWhenNotForeground',%r)\n"
       "unreal.EditorLevelLibrary.set_level_viewport_camera_info(unreal.Vector(*%r),"
       "unreal.Rotator(pitch=%r,yaw=%r,roll=%r))"
       % (state['level'], state['autosave'], state['throttle'],
          state['camera'][:3], state['camera'][3], state['camera'][4], state['camera'][5]))


def wait_port_free(port=41451, timeout_s=120):
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            with socket.socket() as probe:
                probe.bind(('0.0.0.0', port))
            return
        except OSError:
            if time.monotonic() > deadline:
                raise RuntimeError('AirSim RPC port %d not released' % port)
            time.sleep(1)


def start_pie(work, label):
    wait_port_free()
    rows = ue("import unreal\n"
              "print([w.get_path_name() for w in unreal.ObjectIterator(unreal.World) "
              "if 'UEDPIE_' in w.get_path_name()])")
    if any('UEDPIE_' in str(x.get('output', '')) for x in rows['output']):
        raise RuntimeError('editor still holds a PIE world; restart the editor')
    ue("import unreal\n"
       "assert unreal.EditorLevelLibrary.get_game_world() is None\n"
       "unreal.EditorLevelLibrary.load_level(%r)" % LEVEL)
    ue("import unreal\n"
       "w=unreal.EditorLevelLibrary.get_editor_world()\n"
       "assert w.get_world_settings().get_editor_property('DefaultGameMode') is None\n"
       "w.get_world_settings().set_editor_property('DefaultGameMode',"
       "unreal.load_class(None,'/Script/AirSim.AirSimGameMode'))\n"
       "unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorPerformanceSettings'))"
       ".set_editor_property('bThrottleCPUWhenNotForeground',False)\n"
       "unreal.SystemLibrary.execute_console_command(w,'t.MaxFPS 30')\n"
       "unreal.SystemLibrary.execute_console_command(w,'EnableAllScreenMessages')")
    ue('import unreal\nunreal.EditorLevelLibrary.editor_play_simulate()')
    import airsim
    client = None
    names = []
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        try:
            client = airsim.CarClient(timeout_value=2)
            names = client.listVehicles()
            if len(names) == 24:
                break
        except Exception:
            client = None
        time.sleep(.5)
    if client is None or len(names) != 24:
        raise RuntimeError('AirSim fleet unavailable')
    client.stopRecording()
    for name in names:
        client.enableApiControl(True, vehicle_name=name)
        controls = airsim.CarControls()
        controls.brake = 1
        controls.handbrake = True
        client.setCarControls(controls, vehicle_name=name)
    for _ in range(40):
        if max(abs(client.getCarState(vehicle_name=n).speed) for n in names) < .03:
            break
        time.sleep(.5)
    else:
        raise RuntimeError('initial fleet not stationary')
    for name in names:
        client.enableApiControl(False, vehicle_name=name)
    client.simPause(False)
    # The viewport camera is deliberately left untouched: the run must not
    # impose an overhead view. PIE renders from the editor's current camera,
    # which restore_editor() puts back to the operator's own pose afterwards.
    ue("import unreal\n"
       "w=unreal.EditorLevelLibrary.get_game_world()\n"
       "unreal.SystemLibrary.print_string(w,%r,True,False,unreal.LinearColor(0,1,1,1),3600)"
       % ('DEMO STABILITY: ' + label))


def end_pie():
    try:
        ue('import unreal\nunreal.EditorLevelLibrary.editor_end_play()')
    except Exception:
        pass
    time.sleep(1)


def run_trial(work, output):
    out = work / OUT_REL
    command = [sys.executable, str(work / 'scripts/validate_ue_continuous_tasks.py'),
               '--scenario', str(work / SPEC_REL), '--time-scale', '1', '--skip-precheck']
    with (out / 'execution.log').open('w') as log:
        process = subprocess.Popen(command, cwd=work, env=dict(ENV, PYTHONPATH=str(work / 'src')),
                                   stdout=log, stderr=subprocess.STDOUT)
        began = time.monotonic()
        while process.poll() is None:
            if time.monotonic() - began > 4200:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=45)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                raise RuntimeError('trial wall-clock time limit')
            time.sleep(1)
    report = json.loads((out / 'physical_acceptance.json').read_text())
    if report['status'] != 'PASS':
        return dict(status='FAIL', raw_status=report['status'], failure=report.get('failure'),
                    completed_tasks=report.get('completed_tasks'), elapsed_s=report.get('elapsed_s'))
    with (out / 'independent_audits.log').open('w') as log:
        for script, name in (('audit_ue_run.py', 'pose_audit.json'),
                             ('audit_ue_road_envelope.py', 'road_audit.json')):
            subprocess.run([sys.executable, str(work / 'scripts' / script), str(out),
                            '--output', str(out / name)], cwd=work, env=ENV,
                           stdout=log, stderr=subprocess.STDOUT, check=True)
    pose = json.loads((out / 'pose_audit.json').read_text())
    road = json.loads((out / 'road_audit.json').read_text())
    good_parking = (len(pose['parking']) == 16 and all(
        p['position_error_m'] <= .35 and abs(p['heading_error_deg']) <= 5 and abs(p['speed_m_s']) <= .1
        for p in pose['parking']))
    surface = report.get('drivable_surface_audit', {})
    passed = (report.get('completed_tasks') == 32 and report.get('parking_complete') is True
              and len(report.get('parked_nodes', {})) == 16 and good_parking
              and not pose['lane_envelope_violations'] and not road['flags']
              and all(report.get(k) is True for k in ('cleanup_complete', 'fleet_restored', 'api_restored',
                                                     'saved_maps_unchanged', 'saved_geometry_assets_unchanged',
                                                     'native_dependencies_unchanged'))
              and all(report.get(k) == 0 for k in ('vehicle_collisions', 'body_obstacle_contacts',
                                                   'local_unauthorized_entry_count',
                                                   'road_capacity_violation_steps', 'teleports_during_tasks'))
              and surface.get('status') == 'PASS' and surface.get('flag_count') == 0
              and process.returncode == 0)
    return dict(status='PASS' if passed else 'FAIL', raw_status=report.get('status'),
                completed_tasks=report.get('completed_tasks'), parked=len(report.get('parked_nodes', {})),
                elapsed_s=report.get('elapsed_s'), failure=report.get('failure'),
                surface=surface, good_parking=good_parking)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=None, help='new external run directory')
    args = parser.parse_args()
    output = (Path(args.output).expanduser() if args.output else
              OUTPUT_ROOT / time.strftime('ue_acceptance_%Y%m%d_%H%M%S')).resolve()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    started = time.time()
    summary = dict(status='RUNNING', output=str(output), started_wall=started)
    (output).mkdir(parents=True, exist_ok=False)
    lock = (OUTPUT_ROOT / '.acceptance.lock').open('w')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    editor_before = None
    try:
        # Straight to UE on request: no frozen-hash pass, no regression and no
        # native gate. The workspace copy is still isolated outside the repo.
        work = prepare_workspace(output)
        print('工作副本已建立，直接开始真实 UE 验收：%s' % work, flush=True)
        light = dict(status='SKIPPED')
        gate = dict(status='SKIPPED')
        editor_before = save_editor_state(output)
        start_pie(work, output.name)
        try:
            result = run_trial(work, output)
        finally:
            end_pie()
        summary.update(result)
        summary.update(lightweight_status=light['status'], native_gate=gate['status'],
                       editor_restored=True)
    except BaseException as error:
        traceback.print_exc()
        summary.update(status='FAIL', failure=repr(error))
        try:
            end_pie()
        except Exception:
            pass
    finally:
        if editor_before:
            try:
                restore_editor(editor_before)
                summary['editor_restored'] = True
            except Exception as error:
                summary['editor_restore_error'] = repr(error)
        summary['finished_wall'] = time.time()
        (output / 'acceptance_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(json.dumps(summary, indent=2), flush=True)
    return 0 if summary.get('status') == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
