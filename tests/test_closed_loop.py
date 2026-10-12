from dataclasses import replace
import pytest

from logistics_park.simulation.lightweight.simulator import Simulator


@pytest.mark.parametrize("count", [1,2,3,4])
def test_minimal_closed_loop(graph,config,count):
    simulator = Simulator(graph,config,count)
    result = simulator.run()
    assert result["status"] == "LIGHTWEIGHT_SIM_MINIMAL_CLOSED_LOOP_PASS", result
    assert result["control_ticks"] == round(config.duration*config.control_hz)
    assert {row["edge_id"] for row in simulator.rows} == set(config.route)
    assert all(0 <= row["s"] <= graph.edges[row["edge_id"]].length+1e-8 for row in simulator.rows)
    with pytest.raises(RuntimeError):
        simulator.run()


def test_determinism_and_fail_when_insufficient_progress(graph,config):
    config = replace(config,duration=1.0)
    first,second = Simulator(graph,config,2),Simulator(graph,config,2)
    result = first.run()
    assert result == second.run()
    assert result["status"] == "LIGHTWEIGHT_SIM_MINIMAL_CLOSED_LOOP_FAIL"
    assert first.rows == second.rows
    assert first.wraps == {"car_00":0,"car_01":0}


def test_unsafe_spawn_is_rejected(graph,config):
    with pytest.raises(ValueError, match="unsafe initial spawn"):
        Simulator(graph,replace(config,spawn_spacing=0.05),2)


def test_simulation_rate_validation(config):
    with pytest.raises(ValueError):
        replace(config,control_hz=100)
