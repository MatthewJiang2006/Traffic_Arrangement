import json,math
from pathlib import Path
import pytest
from logistics_park.map.turn_geometry import right_turn_path,cardinal_axes
ROOT=Path(__file__).resolve().parents[1]

def test_all_22_right_turns_preserve_lane_endpoints_and_truck_steering_limits():
    base=json.loads((ROOT/'config/topology/industrialarea_ue.json').read_text())
    transport=json.loads((ROOT/'config/topology/industrialarea_ue_transport.json').read_text())
    rows={e['edge_id']:e for e in transport['edges']}
    nodes={n['node_id']:n['position'] for n in base['nodes']}
    turns=[e for e in base['edges'] if e.get('maneuver')=='RIGHT']
    assert len(turns)==22
    for e in turns:
        assert e['incoming_lane'] in ('RIGHT','SINGLE')
        assert e['centerline'][0]==nodes[e['from_node']]
        assert e['centerline'][-1]==nodes[e['to_node']]
        assert rows[e['edge_id']]['centerline']==e['centerline']
        incoming,outgoing=cardinal_axes(e['centerline'])
        assert incoming[0]*outgoing[1]-incoming[1]*outgoing[0]==1
        for a,b,c in zip(e['centerline'],e['centerline'][1:],e['centerline'][2:]):
            ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
            curvature=2*abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/(ab*bc*ac)
            assert curvature<=(1/3.2 if e['incoming_lane']=='RIGHT' else 1/4)+1e-6
        assert e['speed_limit']<=1.5

def test_left_turn_cannot_be_mislabeled_as_right():
    with pytest.raises(ValueError,match='positive yaw'):
        right_turn_path(dict(centerline=[[0,0],[1,0],[6,-5],[6,-6]]))
