from dataclasses import replace
import json
from math import hypot
from pathlib import Path
import pytest

from logistics_park.safety.collision import OBB,overlaps
from logistics_park.simulation.lightweight import fcfs_cli
from logistics_park.simulation.lightweight.fcfs_simulator import TrafficRules

ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'config/simulation/fcfs.json'


@pytest.mark.parametrize('name',['four_way','turns','same_lane_fifo','downstream_full'])
def test_concurrent_fcfs_transport_and_release_tail(name,tmp_path):
    sim,scenario,_,_=fcfs_cli.load_case(CONFIG,name)
    result=sim.run()
    assert result['status']=='PHASE_4_FCFS_CLOSED_LOOP_PASS',result
    assert all(fcfs_cli.acceptance_checks(sim,scenario,result).values())
    assert result['junction_grants']==result['junction_clears']
    assert result['max_translating_vehicles']>=2
    assert result['illegal_intersection_occupancy_vehicle_steps']==0
    assert result['road_capacity_violation_edge_steps']==0
    assert result['fifo_violation_edge_steps']==0
    for edge,count in result['road_capacity_maxima'].items():
        assert count<=sim.graph.edges[edge].storage_capacity
    # Cleared events are triggered only by the full world OBB leaving the zone.
    for event in [e for e in sim.scheduler.events if e['event']=='CLEARED']:
        later=next(row for row in sim.rows if row['vehicle_id']==event['vehicle_id'] and row['timestamp']>=event['time']-1e-9)
        box=OBB(later['x'],later['y'],later['heading'],sim.config.vehicle.length,sim.config.vehicle.width)
        assert not overlaps(box,sim.intersections[event['intersection_id']].box,sim.config.safety_margin)
    if name=='downstream_full':
        row=next(record for record in sim.manager.records.values() if record.task.vehicle_id=='blocker')
        assert row.dispatched_at>=8
        # A blocked entrance does not impede the blocker moving away on its road.
        assert row.completed_at is not None
    if name=='same_lane_fifo':
        grants=[event['vehicle_id'] for event in sim.scheduler.events if event['event']=='GRANTED' and event['intersection_id']=='cross']
        assert grants==['car_02','car_01','car_00']
    sim.write_artifacts(tmp_path,result)
    events=[json.loads(line) for line in (tmp_path/'traffic_events.jsonl').read_text().splitlines()]
    assert events==sim.scheduler.events
    assert json.loads((tmp_path/'summary.json').read_text())['throughput_tasks_per_min']>0


def test_fcfs_is_deterministic_and_timeout_is_fail():
    first,_,_,_=fcfs_cli.load_case(CONFIG,'four_way')
    second,_,_,_=fcfs_cli.load_case(CONFIG,'four_way')
    first.config=replace(first.config,duration=2)
    second.config=replace(second.config,duration=2)
    result=first.run()
    assert result==second.run() and first.rows==second.rows
    assert result['status'].endswith('_FAIL')


def test_waiting_stop_line_and_full_road_occupancy():
    sim,_,_,_=fcfs_cli.load_case(CONFIG,'downstream_full')
    result=sim.run()
    assert result['stop_line_violation_vehicle_steps']==0
    grant=next(event for event in sim.scheduler.events if event['event']=='GRANTED' and event['vehicle_id']=='car_00' and event['intersection_id']=='cross')
    waiting=[row for row in sim.rows if row['vehicle_id']=='car_00' and row['edge_id']=='approach_W' and row['timestamp']<grant['time']]
    limit=sim.graph.edges['approach_W'].length-sim.rules.stop_offset
    assert waiting and all(row['s']<=limit+1e-5 for row in waiting)
    assert any(row['speed']<.005 for row in waiting if row['timestamp']>5)
    assert grant['time']>=8


def test_simulation_step_and_route_changes_never_teleport():
    sim,_,_,_=fcfs_cli.load_case(CONFIG,'four_way')
    sim.config=replace(sim.config,duration=2)
    sim.run()
    for vehicle in sim.states:
        rows=[row for row in sim.rows if row['vehicle_id']==vehicle]
        for a,b in zip(rows,rows[1:]):
            assert hypot(b['x']-a['x'],b['y']-a['y'])<=sim.config.vehicle.v_max*(b['timestamp']-a['timestamp'])+1e-8


def test_invalid_guard_and_failed_cli_return(tmp_path,monkeypatch):
    with pytest.raises(ValueError):
        TrafficRules(stop_offset=.4,request_distance=.35)
    data=json.loads(CONFIG.read_text())
    data['simulation']['duration']=.2
    # Keep relative paths valid even when the manifest is copied elsewhere.
    for case in data['cases']:
        for field in ['map','scenario']:
            case[field]=str((CONFIG.parent/case[field]).resolve())
    path=tmp_path/'fcfs.json'
    path.write_text(json.dumps(data))
    monkeypatch.setattr('sys.argv',['logistics-park-fcfs','--config',str(path),'--output',str(tmp_path/'run')])
    assert fcfs_cli.main()==1
    assert (tmp_path/'run'/'map.json').exists()
