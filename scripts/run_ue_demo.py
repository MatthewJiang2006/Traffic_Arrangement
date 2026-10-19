#!/usr/bin/env python3
"""Record a complete native UE run and restore the editor afterwards."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fps', type=int, default=30)
    parser.add_argument('--scenario', type=Path, default=ROOT/'config/runtime/industrialarea_ue_optimized_20261011.json')
    args = parser.parse_args()
    if not 1 <= args.fps <= 60:
        parser.error('fps must be between 1 and 60')
    scenario = args.scenario.resolve()
    spec = json.loads(scenario.read_text())
    out = ROOT/spec['output']
    out.mkdir(parents=True, exist_ok=True)
    if (out/'physical_acceptance.json').exists():
        raise RuntimeError('Physical report already exists; preserve it before another run')
    state = dict(status='RUNNING', phase='precheck', recording_fps=args.fps)
    recorder = None
    recording_log = None
    editor_saved = False
    started = False

    def save(**values):
        state.update(values)
        (out/'supervision.json').write_text(json.dumps(state, indent=2)+'\n')

    def ue(code):
        result = subprocess.run([sys.executable, str(ROOT/'scripts/ue_remote.py'), code],
                                capture_output=True, text=True, timeout=25)
        with (out/'editor.log').open('a') as log:
            log.write(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError('UE command failed: '+result.stdout+result.stderr)

    try:
        import airsim
        save()
        deadline = time.monotonic()+3600
        while not (out/'lightweight_acceptance.json').exists():
            if time.monotonic()>deadline:
                raise RuntimeError('Precheck time limit')
            time.sleep(2)
        light = json.loads((out/'lightweight_acceptance.json').read_text())
        if light['status']!='PASS' or not light['parking_complete'] or light['completed_tasks']!=len(spec['tasks']):
            raise RuntimeError('Full lightweight precheck failed')
        if any(light.get(k,0) for k in ('collision_count','unauthorized_shared_entry_count','road_capacity_violation_steps','local_unauthorized_entry_count')):
            raise RuntimeError('Unsafe precheck')
        from continuous_run_sources import algorithm_fingerprints
        if light['algorithm_sha256']!=algorithm_fingerprints(ROOT):
            raise RuntimeError('Precheck code differs from current execution')
        save(phase='starting_ue')
        ue("import unreal,builtins; assert unreal.EditorLevelLibrary.get_game_world() is None; assert not unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages(); builtins.optimization_editor_level=unreal.EditorLevelLibrary.get_editor_world().get_path_name().split('.')[0]; builtins.optimization_editor_camera=unreal.EditorLevelLibrary.get_level_viewport_camera_info()")
        editor_saved = True
        ue("import unreal; unreal.EditorLevelLibrary.load_level('/Game/IndustrialArea/Map/IndustrialArea（复件）'); unreal.EditorLevelLibrary.set_selected_level_actors([])")
        ue("import unreal,builtins; w=unreal.EditorLevelLibrary.get_editor_world(); builtins.complete_demo_mode=w.get_world_settings().get_editor_property('DefaultGameMode'); builtins.complete_demo_camera=unreal.EditorLevelLibrary.get_level_viewport_camera_info(); s=unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorPerformanceSettings')); builtins.complete_demo_throttle=s.get_editor_property('bThrottleCPUWhenNotForeground'); s.set_editor_property('bThrottleCPUWhenNotForeground',False); w.get_world_settings().set_editor_property('DefaultGameMode',unreal.load_class(None,'/Script/AirSim.AirSimGameMode'))")
        started = True
        ue('import unreal; unreal.EditorLevelLibrary.editor_play_simulate()')
        for _ in range(30):
            try:
                client = airsim.CarClient(timeout_value=2)
                names = client.listVehicles()
                if len(names)==24:
                    break
            except Exception:
                pass
            time.sleep(.5)
        else:
            raise RuntimeError('AirSim fleet not ready')
        for name in names:
            client.enableApiControl(True, vehicle_name=name)
            controls = airsim.CarControls()
            controls.brake = 1
            controls.handbrake = True
            client.setCarControls(controls, vehicle_name=name)
        for _ in range(30):
            if max(abs(client.getCarState(vehicle_name=n).speed) for n in names)<.03:
                break
            time.sleep(.5)
        else:
            raise RuntimeError('Fleet did not stop before staging')
        for name in names:
            client.enableApiControl(False, vehicle_name=name)
        client.simPause(False)
        ue("import unreal; w=unreal.EditorLevelLibrary.get_game_world(); o=unreal.GameplayStatics.get_world_origin_location(w); unreal.EditorLevelLibrary.set_level_viewport_camera_info(unreal.Vector(-4200-o.x,16000-o.y,22500-o.z),unreal.Rotator(pitch=-90,yaw=90,roll=0))")
        sys.path.insert(0, '/tmp/traffic_fix_ui')
        from Xlib import display, X, protocol
        from PIL import ImageGrab
        import imageio_ffmpeg
        connection = display.Display()
        screen = connection.screen().root
        for identifier in screen.get_full_property(connection.intern_atom('_NET_CLIENT_LIST'), X.AnyPropertyType).value:
            window = connection.create_resource_object('window', int(identifier))
            if 'Traffic_ue4 -' in (window.get_wm_name() or ''):
                screen.send_event(protocol.event.ClientMessage(window=window, client_type=connection.intern_atom('_NET_ACTIVE_WINDOW'), data=(32,[2,X.CurrentTime,0,0,0])), event_mask=X.SubstructureRedirectMask|X.SubstructureNotifyMask)
                connection.flush()
        time.sleep(.8)
        ImageGrab.grab().save(out/'ue_start.png')
        recording_log = (out/'recording.log').open('w')
        recorder = subprocess.Popen([imageio_ffmpeg.get_ffmpeg_exe(), '-y', '-loglevel', 'warning', '-nostats', '-f', 'x11grab', '-framerate', str(args.fps), '-video_size', '1566x814', '-i', os.environ.get('DISPLAY', ':0')+'+488,145', '-an', '-c:v', 'libx264', '-preset', 'ultrafast', '-threads', '2', '-crf', '28', '-pix_fmt', 'yuv420p', str(out/'ue_full_demo.mp4')], stdin=subprocess.DEVNULL, stdout=recording_log, stderr=recording_log)
        save(phase='physical', recording_pid=recorder.pid)
        with (out/'physical.log').open('w') as log:
            result = subprocess.run([sys.executable, '-u', str(ROOT/'scripts/validate_ue_continuous_tasks.py'), '--scenario', str(scenario), '--time-scale', '1'], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        report = json.loads((out/'physical_acceptance.json').read_text())
        save(phase='finished', status=report['status'], completed_tasks=report.get('completed_tasks'), elapsed_s=report.get('elapsed_s'), failure=report.get('failure'), physical_returncode=result.returncode)
        ImageGrab.grab().save(out/'ue_end.png')
    except BaseException as error:
        save(status='FAIL', phase='failed', failure=repr(error))
        traceback.print_exc()
    finally:
        if recorder is not None:
            if recorder.poll() is None:
                recorder.send_signal(signal.SIGINT)
                try:
                    recorder.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    recorder.terminate()
                    recorder.wait(timeout=10)
            save(recording_returncode=recorder.returncode)
        if recording_log:
            recording_log.close()
        if started:
            try:
                ue('import unreal; unreal.EditorLevelLibrary.editor_end_play()')
                time.sleep(1)
                ue("import unreal,builtins; w=unreal.EditorLevelLibrary.get_editor_world(); w.get_world_settings().set_editor_property('DefaultGameMode',builtins.complete_demo_mode); unreal.EditorLevelLibrary.set_level_viewport_camera_info(*builtins.complete_demo_camera); unreal.get_default_object(unreal.load_class(None,'/Script/UnrealEd.EditorPerformanceSettings')).set_editor_property('bThrottleCPUWhenNotForeground',builtins.complete_demo_throttle)")
            except Exception as error:
                save(editor_restore_error=repr(error))
        if editor_saved:
            try:
                ue("import unreal,builtins; unreal.EditorLevelLibrary.load_level(builtins.optimization_editor_level); unreal.EditorLevelLibrary.set_level_viewport_camera_info(*builtins.optimization_editor_camera)")
                save(editor_restored='editor_restore_error' not in state)
            except Exception as error:
                save(editor_restore_error=repr(error))
    print(json.dumps(state, ensure_ascii=False), flush=True)
    return 0 if state['status']=='PASS' and state.get('editor_restored') else 1


if __name__=='__main__':
    raise SystemExit(main())
