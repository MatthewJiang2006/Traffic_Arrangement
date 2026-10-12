from dataclasses import replace
import pytest

from logistics_park.common.enums import NodeType,TaskStatus
from logistics_park.common.messages import Task
from logistics_park.map.road_graph import Node,RoadGraph
from logistics_park.tasks.service_station import ServiceStation,StationConfig
from logistics_park.tasks.task_manager import TaskManager
from logistics_park.tasks.scenario_generator import cyclic_tasks,fixed_tasks,random_tasks


def test_capacity_commitments_fifo_service_and_physical_departure(task_graph,task_scenario):
    homes,stations,_ = task_scenario
    manager = TaskManager(task_graph,stations,homes)
    for i in range(4):
        manager.submit(Task(f"t{i}",f"car_{i:02d}",f"G{i}","W"))
    dispatches = manager.dispatch_ready(0)
    assert len(dispatches) == 3
    assert manager.records["t3"].status == TaskStatus.PENDING
    assert manager.stations["W"].max_reserved == 3
    assert "car_03" not in manager.departures  # stays parked at home
    for i,dispatch in enumerate(dispatches):
        manager.clear_departure(dispatch.vehicle_id)
        manager.arrive(dispatch.vehicle_id,dispatch.route.destination,i+1)
    assert manager.stations["W"].max_waiting == 2
    assert manager.records["t0"].status == TaskStatus.SERVICING
    assert manager.records["t1"].status == TaskStatus.ARRIVED
    manager.advance(40.99)
    assert manager.records["t0"].status == TaskStatus.SERVICING
    manager.advance(41)
    assert manager.records["t0"].status == TaskStatus.TASK_COMPLETE
    assert manager.records["t1"].service_started_at == 41
    assert len(manager.stations["W"].occupants) == 3  # completed is still parked
    manager.submit(Task("z_return","car_00","W","G0"))
    assert [item.task_id for item in manager.dispatch_ready(41)] == ["z_return"]
    assert len(manager.stations["W"].occupants) == 3  # dispatch itself must not free the berth
    assert manager.dispatch_ready(41.1) == []
    manager.clear_departure("car_00")
    assert [item.task_id for item in manager.dispatch_ready(41.2)] == ["t3"]
    assert manager.records["t3"].dispatched_at == 41.2


def test_vehicle_itinerary_precedes_priority_and_release_time(task_graph,task_scenario):
    homes,stations,_ = task_scenario
    manager = TaskManager(task_graph,stations,homes)
    manager.submit(Task("first","car_00","G0","W",release_time=10))
    manager.submit(Task("second","car_00","W","G0",release_time=0,priority=99))
    assert manager.dispatch_ready(0) == []
    assert [item.task_id for item in manager.dispatch_ready(10)] == ["first"]
    manager.clear_departure("car_00")
    with pytest.raises(ValueError):
        manager.arrive("car_00","wrong",11)
    record = manager.records["first"]
    manager.arrive("car_00",record.target_node,11)
    assert manager.dispatch_ready(11) == []  # service unfinished
    assert [item.task_id for item in manager.dispatch_ready(51)] == ["second"]


def test_origin_mismatch_and_submission_rejections(task_graph,task_scenario):
    homes,stations,_ = task_scenario
    manager = TaskManager(task_graph,stations,homes)
    task = Task("bad_origin","car_00","S","W")
    manager.submit(task)
    assert manager.dispatch_ready(0) == []
    assert manager.records[task.task_id].status == TaskStatus.FAILED
    assert manager.records[task.task_id].failure_reason
    assert manager.stations["W"].occupants == {}
    for bad in [task,Task("unknown_vehicle","missing","G0","W"),
                Task("unknown_station","car_00","G0","missing"),
                Task("private_berth","car_00","G0","w0")]:
        with pytest.raises(ValueError):
            manager.submit(bad)
    with pytest.raises(ValueError):
        manager.advance(-1)


def test_unreachable_destination_does_not_leak_reservations(task_graph):
    graph = RoadGraph(list(task_graph.nodes.values())+[Node("island",NodeType.WAREHOUSE,(10,10))],
                      list(task_graph.edges.values()),list(task_graph.movements.values()))
    config = StationConfig("isolated",("island",),waiting_capacity=0)
    manager = TaskManager(graph,[config],{"v":"G0"})
    manager.submit(Task("unreachable","v","G0","isolated"))
    assert manager.dispatch_ready(0) == []
    assert manager.records["unreachable"].status == TaskStatus.FAILED
    assert manager.stations["isolated"].occupants == {} and manager.active == {}
    # An unreachable first berth must not hide a reachable second one.
    config = StationConfig("mixed",("island","w0"),waiting_capacity=1)
    manager = TaskManager(graph,[config],{"v":"G0"})
    manager.submit(Task("reachable","v","G0","mixed"))
    dispatch = manager.dispatch_ready(0)[0]
    assert dispatch.route.destination == "w0"


def test_service_station_zero_time_and_multiple_servers():
    station = ServiceStation(StationConfig("gate",("a","b","c"),service_capacity=2,waiting_capacity=1,service_time=0))
    for i,node in enumerate(("a","b","c")):
        station.reserve(str(i),f"t{i}",node)
        station.arrive(str(i),0)
    started,completed = station.advance(0)
    assert len(started) == len(completed) == 2
    assert station.max_servicing == 2 and station.max_waiting == 1
    assert len(station.advance(0)[1]) == 1
    with pytest.raises(ValueError):
        station.reserve("0","duplicate")
    with pytest.raises(ValueError):
        StationConfig("bad",("a",),waiting_capacity=2)


def test_generators_are_reproducible_and_preserve_round_trips():
    homes = {"v1":"G1","v0":"G0"}
    first = random_tasks(homes,("W","S"),3,2026,release_interval=2)
    assert first == random_tasks(dict(reversed(list(homes.items()))),("W","S"),3,2026,release_interval=2)
    assert len({task.task_id for task in first}) == len(first) == 12
    for out,back in zip(first[::2],first[1::2]):
        assert out.vehicle_id == back.vehicle_id and out.destination == back.origin
        assert out.origin == back.destination
    cyclic = cyclic_tasks({"v":"G0"},("W","S"),2)
    assert [(task.origin,task.destination) for task in cyclic] == [("G0","W"),("W","S"),("S","G0")]*2
    assert fixed_tasks([{"task_id":"f","vehicle_id":"v","origin":"G0","destination":"W"}])[0].task_id == "f"
    for generate in [lambda: random_tasks(homes,(),1,0),lambda: random_tasks(homes,("W",),0,0),
                     lambda: cyclic_tasks(homes,("W",),0),lambda: cyclic_tasks({"v":"W"},("W",),1)]:
        with pytest.raises(ValueError):
            generate()
