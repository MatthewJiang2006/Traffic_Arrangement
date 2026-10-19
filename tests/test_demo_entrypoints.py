"""Keep the Demo launcher and viewport progress working after code alignment."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest


ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / (name + '.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('failed_stage,expected_calls', [(None, 1), (0, 1)])
def test_launcher_runs_ue_acceptance_directly_with_external_output(monkeypatch, tmp_path,
                                                                   failed_stage, expected_calls):
    entry = load_script('run')
    calls = []
    lifecycle = []

    def child(command, cwd):
        calls.append((command, cwd))
        return SimpleNamespace(returncode=7 if failed_stage is not None else 0)

    monkeypatch.setattr(entry.subprocess, 'run', child)
    monkeypatch.setattr(entry, 'prepare_scratch', lambda output: None)
    monkeypatch.setattr(entry, 'start_pie', lambda: lifecycle.append('start'))
    monkeypatch.setattr(entry, 'end_pie', lambda: lifecycle.append('stop'))
    monkeypatch.setattr(entry, 'DEFAULT_OUTPUT_ROOT', tmp_path / 'outside')
    monkeypatch.setattr(entry.sys, 'argv', ['run.py'])
    monkeypatch.chdir(tmp_path)
    assert entry.main() == (0 if failed_stage is None else 7)
    assert lifecycle == ['start', 'stop']
    assert len(calls) == expected_calls
    command, cwd = calls[0]
    assert command[0] == entry.sys.executable
    assert cwd == ROOT
    scenario = Path(command[command.index('--scenario') + 1])
    assert scenario == ROOT / 'config/runtime/industrialarea_ue_full_verified_20261007.json'
    assert scenario.is_file()
    assert '--skip-precheck' in command
    assert command[command.index('--time-scale') + 1] == '1'


def test_launcher_output_defaults_outside_the_repository(monkeypatch, tmp_path):
    entry = load_script('run')
    seen = []
    monkeypatch.setattr(entry.subprocess, 'run',
                        lambda command, cwd: seen.append(command) or SimpleNamespace(returncode=0))
    monkeypatch.setattr(entry, 'prepare_scratch', lambda output: None)
    monkeypatch.setattr(entry, 'start_pie', lambda: None)
    monkeypatch.setattr(entry, 'end_pie', lambda: None)
    monkeypatch.setattr(entry, 'DEFAULT_OUTPUT_ROOT', tmp_path / 'outside')
    monkeypatch.setattr(entry.sys, 'argv', ['run.py'])
    assert entry.main() == 0
    output = Path(seen[0][seen[0].index('--output') + 1])
    assert output.parent == (tmp_path / 'outside').resolve()
    assert ROOT not in output.parents


class Client:
    def __init__(self):
        self.recording = True
        self.messages = []

    def stopRecording(self):
        self.recording = False

    def simRunConsoleCommand(self, command):
        return command == 'EnableAllScreenMessages'

    def simPrintLogMessage(self, key, text, severity):
        self.messages.append((key, text, severity))


def test_progress_stops_recording_and_displays_transport_parking_and_final_failure():
    client = Client()
    progress = load_script('ue_demo_progress').DemoProgress(client, 32, 16)
    progress.start()
    assert not client.recording
    assert 'PREPARING' in client.messages[-1][1]
    progress.update('RUNNING', 16, 5, 2, 61)
    assert 'Tasks 16/32 (50%)' in client.messages[-1][1]
    assert 'Moving 5/16' in client.messages[-1][1]
    assert 'Elapsed 01:01' in client.messages[-1][1]
    count = len(client.messages)
    progress.update('RUNNING', 17, elapsed=61.5)
    assert len(client.messages) == count
    progress.update('PARKING', 32, 1, 15, 62)
    assert 'PARKING' in client.messages[-1][1]
    assert 'Parked 15/16' in client.messages[-1][1]
    progress.update('FAIL', 32, 0, 15, 62.1, failure='blocked exit', force=True)
    assert client.messages[-1][2] == 2
    assert 'blocked exit' in client.messages[-1][1]
    assert len({message[0] for message in client.messages}) == 1
