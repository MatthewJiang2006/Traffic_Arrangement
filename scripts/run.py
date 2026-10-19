#!/usr/bin/env python3
"""Run the verified full UE demo directly: start PIE, run acceptance, restore.

No lightweight precheck. Acceptance data is written outside the repository by
default, so the working tree never receives trajectories, logs or reports.
"""

import argparse
import fcntl
import json
import os
import shutil
import socket
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT / 'config/runtime/industrialarea_ue_full_verified_20261007.json'
LEVEL = '/Game/IndustrialArea/Map/map'
DEFAULT_OUTPUT_ROOT = Path(os.environ.get('DEMO_OUTPUT_ROOT', ROOT.parent / 'Demo_runs'))


def ue(code):
    """Run Python inside the UE editor through the remote-execution helper."""
    result = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/ue_remote.py'), 'exec(' + repr(code) + ', {})'],
        capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError('UE remote command failed: ' + result.stderr[-2000:])
    return result.stdout




def run_is_active(target):
    """True while another acceptance still holds the run's scratch lock."""
    lock = target / 'continuous_physical.lock'
    if not lock.is_file():
        return False
    with lock.open() as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            return True
        fcntl.flock(handle, fcntl.LOCK_UN)
    return False


def prepare_scratch(output):
    """Point the UE scripts' fixed runs/ paths at the external output dir.

    The editor-side scripts (export_scene, monitor_fleet, ...) write to
    ROOT/runs/... by design; symlinks keep that contract while the actual
    data stays outside the repository.
    """
    runs = ROOT / 'runs'
    runs.mkdir(exist_ok=True)
    for name in ('ue_scene', 'ue_node_pairs', 'continuous_tasks'):
        target = output / name
        target.mkdir(parents=True, exist_ok=True)
        link = runs / name
        if link.is_symlink():
            if link.resolve() == target.resolve():
                continue
            if run_is_active(link.resolve()):
                raise RuntimeError('runs/%s is owned by a live run: %s' % (name, link.resolve()))
            link.unlink()
        elif link.exists():
            shutil.rmtree(link)
        link.symlink_to(target)



def ue_eval(code):
    """Run Python in the editor and return its printed lines."""
    result = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/ue_remote.py'), 'exec(' + repr(code) + ', {})'],
        capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError('UE remote command failed: ' + result.stderr[-2000:])
    row = json.loads(result.stdout)
    return [str(item.get('output', '')) for item in row.get('output', [])]


def assert_clean_editor():
    """Refuse to start on an editor that still holds a PIE world.

    Repeated PIE sessions leave stale worlds and duplicated truck pawns behind;
    every later trial then reads the wrong world and fails for the wrong reason.
    """
    lines = ue_eval("import unreal\n"
                    "print([w.get_path_name() for w in unreal.ObjectIterator(unreal.World) "
                    "if 'UEDPIE_' in w.get_path_name()])")
    if any('UEDPIE_' in line for line in lines):
        raise RuntimeError('editor still holds a PIE world; restart the editor before running')


def prepare_scratch(output):
    """Point the UE scripts' fixed runs/ paths at the external output dir.

    The editor-side scripts (export_scene, monitor_fleet, ...) write to
    ROOT/runs/... by design; symlinks keep that contract while the actual
    data stays outside the repository.
    """
    runs = ROOT / 'runs'
    runs.mkdir(exist_ok=True)
    for name in ('ue_scene', 'ue_node_pairs', 'continuous_tasks'):
        target = output / name
        target.mkdir(parents=True, exist_ok=True)
        link = runs / name
        if link.is_symlink():
            if link.resolve() == target.resolve():
                continue
            if run_is_active(link.resolve()):
                raise RuntimeError('runs/%s is owned by a live run: %s' % (name, link.resolve()))
            link.unlink()
        elif link.exists():
            shutil.rmtree(link)
        link.symlink_to(target)



def ue_eval(code):
    """Run Python in the editor and return its printed lines."""
    result = subprocess.run(
        [sys.executable, str(ROOT / 'scripts/ue_remote.py'), 'exec(' + repr(code) + ', {})'],
        capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError('UE remote command failed: ' + result.stderr[-2000:])
    row = json.loads(result.stdout)
    return [str(item.get('output', '')) for item in row.get('output', [])]


def assert_clean_editor():
    """Refuse to start on an editor that still holds a PIE world.

    Repeated PIE sessions leave stale worlds and duplicated truck pawns behind;
    every later trial then reads the wrong world and fails for the wrong reason.
    """
    lines = ue_eval("import unreal\n"
                    "print([w.get_path_name() for w in unreal.ObjectIterator(unreal.World) "
                    "if 'UEDPIE_' in w.get_path_name()])")
    if any('UEDPIE_' in line for line in lines):
        raise RuntimeError('editor still holds a PIE world; restart the editor before running')


def assert_clean_fleet(names):
    if len(names) != 24 or len(set(names)) != 24:
        raise RuntimeError('expected 24 distinct AirSim vehicles, got %d' % len(names))
    lines = ue_eval("import unreal\n"
                    "w=next(w for w in unreal.ObjectIterator(unreal.World) if 'UEDPIE_' in w.get_path_name())\n"
                    "actors=unreal.GameplayStatics.get_all_actors_of_class(w,unreal.Actor)\n"
                    "cars=[a.get_name() for a in actors if a.get_name().startswith('Car_')]\n"
                    "print(len(cars), len(set(cars)))")
    numbers = [int(x) for x in __import__('re').findall(r'\d+', lines[-1])] if lines else []
    if numbers != [24, 24]:
        raise RuntimeError('PIE world holds duplicated or missing trucks: %s' % numbers)


def wait_until_port_free(port=41451, timeout_s=120):
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


def start_pie():
    """Load the map, enter PIE and stop the fresh fleet before acceptance."""
    wait_until_port_free()
    assert_clean_editor()
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
    # A client whose first connection failed can stay unusable, so build a
    # fresh one on every retry while the AirSim RPC server is still binding.
    client = None
    names = []
    deadline = time.monotonic() + 180
    while time.monotonic() < deadline:
        try:
            client = airsim.CarClient(timeout_value=2)
            names = client.listVehicles()
            if len(names) == 24:
                break
        except Exception:
            client = None
        time.sleep(1)
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
        raise RuntimeError('Initial fleet not stationary')
    for name in names:
        client.enableApiControl(False, vehicle_name=name)
    client.simPause(False)
    # Let the skeletal-mesh physics instances initialise before the acceptance
    # freezes the fleet; a fresh PIE world needs a couple of ticks first.
    time.sleep(3)


def end_pie():
    """Leave PIE and reload the saved map so the editor is reusable."""
    try:
        ue('import unreal\nunreal.EditorLevelLibrary.editor_end_play()')
    finally:
        time.sleep(1)
        ue("import unreal\n"
           "w=unreal.EditorLevelLibrary.get_editor_world()\n"
           "w.get_world_settings().set_editor_property('DefaultGameMode',None)\n"
           "unreal.EditorLevelLibrary.load_level(%r)" % LEVEL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default=None,
                        help='run directory (default: DEMO_OUTPUT_ROOT/ue_full_<timestamp>)')
    parser.add_argument('--time-scale', default='1', choices=('1', '2', '4'))
    args = parser.parse_args()
    output = Path(args.output).expanduser() if args.output else (
        DEFAULT_OUTPUT_ROOT / time.strftime('ue_full_%Y%m%d_%H%M%S'))
    output = output.resolve()
    print('直接 UE 验收（%s 倍速），输出目录: %s' % (args.time_scale, output), flush=True)
    try:
        prepare_scratch(output)
        start_pie()
        result = subprocess.run(
            [sys.executable, str(ROOT / 'scripts/validate_ue_continuous_tasks.py'),
             '--scenario', str(SCENARIO), '--time-scale', args.time_scale,
             '--output', str(output), '--skip-precheck'],
            cwd=ROOT)
    finally:
        try:
            end_pie()
        except Exception as exc:
            print('编辑器还原失败:', exc, file=sys.stderr, flush=True)
    if result.returncode:
        print('UE 验收失败。', file=sys.stderr, flush=True)
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
