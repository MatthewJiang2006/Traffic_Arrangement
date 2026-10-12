from pathlib import Path
import pytest

from logistics_park.map.map_loader import load_map
from logistics_park.simulation.lightweight.simulator import SimulationConfig

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def graph():
    return load_map(ROOT/"config/topology/minimal_loop.json")


@pytest.fixture
def config():
    return SimulationConfig.load(ROOT/"config/simulation/minimal.json")


@pytest.fixture
def task_graph():
    return load_map(ROOT/"config/topology/task_park.json")


@pytest.fixture
def task_config():
    from logistics_park.simulation.lightweight.task_simulator import TaskSimulationConfig
    return TaskSimulationConfig.load(ROOT/"config/simulation/tasks.json")


@pytest.fixture
def task_scenario():
    from logistics_park.simulation.lightweight.task_cli import load_scenario
    return load_scenario(ROOT/"config/tasks/phase_3.json","fixed")
