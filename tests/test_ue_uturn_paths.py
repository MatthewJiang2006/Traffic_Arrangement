"""Geometry regression for the authorized terminal U-turn fixtures."""
import importlib.util
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('ue_uturn_paths',ROOT/'scripts/ue_uturn_paths.py')
paths_module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(paths_module)

def paths():
    return paths_module.make_paths(json.loads((ROOT/'config/topology/industrialarea_ue_survey.json').read_text()))

def curvature(a,b,c):
    ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
    return 2*abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))/(ab*bc*ac) if min(ab,bc,ac)>1e-8 else 0

def test_wide_road_uturns_leave_on_opposite_right_lane():
    sites=json.loads((ROOT/'config/traffic/industrialarea_ue_test_sites.json').read_text())['sites']
    for label,sign in [('A',1),('C',-1)]:
        path=paths()[label][0]['rear_path']
        start=sites[label]['start']['xy_m'];end=sites[label]['finish']['xy_m']
        assert math.dist(path[0],[start[0],start[1]-sign*paths_module.WHEELBASE/2])<1e-6
        assert math.dist(path[-1],[end[0],end[1]+sign*paths_module.WHEELBASE/2])<1e-6
        assert max(curvature(a,b,c) for a,b,c in zip(path,path[1:],path[2:]))<math.tan(math.radians(40))/paths_module.WHEELBASE

def test_three_arc_paths_end_at_opposite_heading_without_reverse():
    for label in ['B','D']:
        for plan in paths()[label]:
            path=plan['rear_path']
            assert path[1][0]>path[0][0]
            assert path[-1][0]<path[-2][0]
            assert abs(path[0][1]-path[-1][1]-3.8)<1e-6
            assert max(curvature(a,b,c) for a,b,c in zip(path,path[1:],path[2:]))<math.tan(math.radians(40))/paths_module.WHEELBASE

def test_ccc_geometry_preserves_endpoints_and_turn_radius():
    for sign in [-1,1]:
        for side in [-1,1]:
            p=paths_module.ccc([0,1.9],[0,-1.9],0,3.2,sign,side)
            assert math.dist(p[0],[0,1.9])<1e-8
            assert math.dist(p[-1],[0,-1.9])<1e-8
            assert max(curvature(a,b,c) for a,b,c in zip(p,p[1:],p[2:]))<=1/3.2+1e-7
