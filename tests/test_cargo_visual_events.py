import json
from logistics_park.runtime.cargo_visuals import CargoVisualEvents


def test_station_service_controls_visuals_once_in_order(tmp_path):
    path=tmp_path/'events.json';path.write_text('{"commands":[]}')
    bridge=CargoVisualEvents(path)
    events=[dict(event='ARRIVED',vehicle_id='truck',site='W5-loading',time=1),
            dict(event='SERVICING',vehicle_id='truck',site='W5-loading',operation='LOADING',time=1)]
    bridge.update(events);bridge.update(events)
    events.extend([dict(event='LOADING_COMPLETE',vehicle_id='truck',time=6),
                   dict(event='SERVICING',vehicle_id='truck',site='W6-unloading',operation='UNLOADING',time=10),
                   dict(event='SERVICING',vehicle_id='truck',site='Q2',operation=None,time=20),
                   dict(event='SERVICING',vehicle_id='truck',site='W5-loading',operation='LOADING',time=25)])
    bridge.update(events)
    commands=json.loads(path.read_text())['commands']
    assert [(c['id'],c['operation'],c['service_time_s']) for c in commands]==[(0,'LOADING',1),(1,'UNLOADING',10),(2,'LOADING',25)]
    assert not path.with_suffix('.tmp').exists()
