import csv
import json
from dataclasses import replace
from pathlib import Path
import pytest

from logistics_park.common.enums import TaskStatus
from logistics_park.simulation.lightweight import task_cli
from logistics_park.simulation.lightweight.task_simulator import TaskSimulator

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize("algorithm",["dijkstra","astar"])
@pytest.mark.parametrize("mode",["fixed","random","cyclic"])
def test_transport_lifecycle_and_safety(task_graph,task_config,algorithm,mode):
    homes,stations,tasks = task_cli.load_scenario(ROOT/"config/tasks/phase_3.json",mode)
    simulator = TaskSimulator(task_graph,replace(task_config,algorithm=algorithm),stations,homes,tasks)
    result = simulator.run()
    assert result["status"] == "PHASE_3_TASK_CLOSED_LOOP_PASS",result
    assert result["completed_tasks"] == len(tasks)
    assert result["max_moving_vehicles"] <= 1
    for record in simulator.manager.records.values():
        assert record.task.release_time <= record.dispatched_at <= record.arrived_at <= record.service_started_at <= record.completed_at
        service_time = simulator.manager.stations[record.task.destination].config.service_time
        assert record.completed_at-record.service_started_at >= service_time-1e-8
        events = [event["status"] for event in simulator.manager.events if event["task_id"] == record.task.task_id]
        assert events == ["PENDING","EN_ROUTE","ARRIVED","SERVICING","TASK_COMPLETE"]
    if mode == "fixed":
        assert result["stations"]["W"]["max_reserved"] == 3
        assert result["stations"]["W"]["max_waiting"] == 2
        assert result["admission_denial_attempts"] > 0
        fourth = simulator.manager.records["fixed_3_out"]
        first_complete = simulator.manager.records["fixed_0_out"].completed_at
        assert fourth.dispatched_at > first_complete
        held_rows = [row for row in simulator.rows if row["vehicle_id"] == "car_03" and row["timestamp"] < fourth.dispatched_at]
        start = task_graph.nodes[homes["car_03"]].position
        assert all((row["x"],row["y"]) == start and row["speed"] == 0 for row in held_rows)
    # Every vehicle returns home through actual motion, without pose snapping.
    for vehicle,node in homes.items():
        state = simulator.states[vehicle]
        x,y = task_graph.nodes[node].position
        assert ((state.x-x)**2+(state.y-y)**2)**.5 <= task_config.arrival_tolerance
    with pytest.raises(RuntimeError):
        simulator.run()


def test_task_trajectory_has_no_teleportation_and_records_are_reproducible(task_graph,task_config,task_scenario,tmp_path):
    homes,stations,tasks = task_scenario
    config = replace(task_config,duration=3)
    first,second = [TaskSimulator(task_graph,config,stations,homes,tasks) for _ in range(2)]
    result = first.run()
    assert result == second.run()
    assert result["status"].endswith("_FAIL")
    assert first.rows == second.rows
    for vehicle in homes:
        rows = [row for row in first.rows if row["vehicle_id"] == vehicle]
        for before,after in zip(rows,rows[1:]):
            distance = ((after["x"]-before["x"])**2+(after["y"]-before["y"])**2)**.5
            assert distance <= config.vehicle.v_max*(after["timestamp"]-before["timestamp"])+1e-8
    first.write_artifacts(tmp_path,result)
    assert json.loads((tmp_path/"summary.json").read_text()) == result
    records = [json.loads(line) for line in (tmp_path/"tasks.jsonl").read_text().splitlines()]
    assert len(records) == len(tasks)
    assert (tmp_path/"events.jsonl").read_text()
    with (tmp_path/"trajectory.csv").open() as stream:
        assert "task_status" in next(csv.DictReader(stream))


def test_task_cli_failed_episode_returns_nonzero(task_config,tmp_path,monkeypatch):
    data = json.loads((ROOT/"config/simulation/tasks.json").read_text())
    data["duration"] = .2
    path = tmp_path/"config.json"
    path.write_text(json.dumps(data))
    monkeypatch.setattr("sys.argv",["logistics-park-tasks","--map",str(ROOT/"config/topology/task_park.json"),
                                   "--config",str(path),"--scenario",str(ROOT/"config/tasks/phase_3.json"),
                                   "--output",str(tmp_path/"run")])
    assert task_cli.main() == 1
    assert (tmp_path/"run"/"map.json").exists()
    assert (tmp_path/"run"/"scenario.json").exists()


def test_invalid_rates_and_overlapping_initial_parking(task_graph,task_config,task_scenario):
    homes,stations,tasks = task_scenario
    with pytest.raises(ValueError):
        replace(task_config,task_hz=100)
    with pytest.raises(ValueError):
        replace(task_config,algorithm="unknown")
    with pytest.raises(ValueError,match="distinct initial"):
        TaskSimulator(task_graph,task_config,stations,{**homes,"car_01":"G0"},tasks)
