from copy import deepcopy
import pytest

from logistics_park.runtime.ground_contact import supported_ramp_contact


def road_seam_observation():
    return dict(collision='SM_Road_142',collision_normal=[-.4318378,.3002657,-.8505038],
        collision_penetration_m=.00402156,
        wheel_clearances_m=[-.0162848,-.0742291,.0549536,.1263452,.0352594,-.0361322],
        wheel_surfaces=['SM_Road_142']+['SM_Road_20']*5,
        body_hits=[None]*3,parked_vehicle_overlaps=[None]*3)


def test_observed_q37_shallow_tire_seam_support_is_not_a_chassis_obstacle():
    row=road_seam_observation();surfaces={'SM_Road_142','SM_Road_20'}
    assert supported_ramp_contact(row,surfaces,clearance_m=None)
    assert not supported_ramp_contact(row,surfaces,clearance_m=.12)


@pytest.mark.parametrize('change',[
    {'body_hits':['SM_Wall_1',None,None]},
    {'parked_vehicle_overlaps':['other',None,None]},
    {'collision_normal':[1,0,0]},
    {'collision_normal':[.6,0,-.8]},
    {'collision_penetration_m':.02},
    {'wheel_surfaces':['SM_Road_142']*6},
    {'wheel_clearances_m':[.25]*6},
])
def test_road_seam_exception_keeps_actual_obstacles_and_large_impacts_rejected(change):
    row=road_seam_observation();row.update(change)
    assert not supported_ramp_contact(row,{'SM_Road_142','SM_Road_20'},clearance_m=None)

RAMPS = {"TrafficW1RoadToCurb"}


def observed_support():
    # W1 northbound trial: vertical surface support with no visible body hit.
    return dict(collision="TrafficW1RoadToCurb",
                collision_normal=[.0316, -.06, -.9977], collision_penetration_m=-.0377,
                wheel_surfaces=["TrafficW1RoadToCurb"]*2+["SM_Road_62"]*4,
                wheel_clearances_m=[.0116,-.0032,-.0515,-.0896,-.0442,-.0061],
                body_hits=[None]*3, parked_vehicle_overlaps=[None]*3)


def test_supported_contact_is_distinct_from_collision():
    assert supported_ramp_contact(observed_support(), RAMPS)


@pytest.mark.parametrize("change", [
    {"collision_normal": [1,0,0]},
    {"collision_penetration_m": .05},
    {"wheel_clearances_m": [.13]*6},
    {"wheel_clearances_m": [None]*6},
    {"body_hits": ["TrafficW1RoadToCurb",None,None]},
    {"body_hits": []},
    {"parked_vehicle_overlaps": ["other",None,None]},
    {"collision": "SM_Wall_156"},
    {"wheel_surfaces": ["SM_Road_62"]*6},
])
def test_unsafe_or_incomplete_contact_is_rejected(change):
    row = deepcopy(observed_support())
    row.update(change)
    assert not supported_ramp_contact(row, RAMPS)


def test_project_mode_allows_other_wheels_to_lift():
    row = observed_support()
    row["wheel_clearances_m"] = [.01,.01,.2,.2,None,.2]
    assert supported_ramp_contact(row, RAMPS, clearance_m=None)
    row["body_hits"][0] = "SM_Wall_156"
    assert not supported_ramp_contact(row, RAMPS, clearance_m=None)


def test_project_mode_still_requires_a_supported_contacting_wheel():
    row = observed_support()
    row["wheel_clearances_m"] = [.2]*6
    assert not supported_ramp_contact(row, RAMPS, clearance_m=None)


def test_surveyed_road_support_keeps_lateral_and_body_contacts_unsafe():
    row = observed_support()
    row["collision"] = "SM_Road_63"
    row["wheel_surfaces"][:2] = ["SM_Road_63"]*2
    surfaces = RAMPS | {"SM_Road_63"}
    assert supported_ramp_contact(row, surfaces, clearance_m=None)
    row["collision_normal"] = [1,0,0]
    assert not supported_ramp_contact(row, surfaces, clearance_m=None)
    row["collision_normal"] = [0,0,-1]
    row["body_hits"][0] = "SM_Road_63"
    assert not supported_ramp_contact(row, surfaces, clearance_m=None)


def test_observed_road_edge_tire_support_is_accepted_but_side_contact_is_not():
    row = observed_support()
    row.update(collision="SM_Road_87", collision_normal=[.32076,0,-.94716],
               collision_penetration_m=.000083, wheel_surfaces=["SM_Road_87"]*6,
               wheel_clearances_m=[-.09086,.01195,-.00596,-.08969,.01098,-.05529])
    assert supported_ramp_contact(row,{"SM_Road_87"},clearance_m=None)
    row["collision_normal"]=[.6,0,-.8]
    assert not supported_ramp_contact(row,{"SM_Road_87"},clearance_m=None)


def test_adjacent_ramp_contact_requires_known_ground_and_clear_body():
    row=observed_support()
    row.update(collision="TrafficWaitingQ11Q12CurbToHardstand",
               collision_normal=[.01754,0,-.99985],collision_penetration_m=.000063,
               wheel_surfaces=["SM_Road_41","SM_Road_41","SM_Road_40","SM_Road_51","SM_Road_41","SM_Road_40"],
               wheel_clearances_m=[-.0731,.0414,.0552,.1059,.0476,-.003])
    known={row["collision"],"SM_Road_41","SM_Road_40","SM_Road_51"}
    assert supported_ramp_contact(row,known,clearance_m=None)
    assert not supported_ramp_contact(row,known)
    row["wheel_surfaces"][0]="SM_Fence_1"
    assert not supported_ramp_contact(row,known,clearance_m=None)
    row["wheel_surfaces"][0]="SM_Road_41"
    row["body_hits"][0]=row["collision"]
    assert not supported_ramp_contact(row,known,clearance_m=None)


def test_project_mode_accepts_shallow_tire_ground_contact_without_body_hit():
    row=observed_support()
    row.update(collision="SM_Road_51",collision_normal=[0,-.064,-.998],
               collision_penetration_m=.0235,wheel_surfaces=["SM_Road_51"]*6,
               wheel_clearances_m=[.009,-.0475,.059,-.013,-.04,0])
    known={"SM_Road_51"}
    assert supported_ramp_contact(row,known,clearance_m=None)
    assert not supported_ramp_contact(row,known)
    row["collision_penetration_m"]=.09
    assert not supported_ramp_contact(row,known,clearance_m=None)
    row["collision_penetration_m"]=.0235
    row["body_hits"][0]="SM_Road_51"
    assert not supported_ramp_contact(row,known,clearance_m=None)


def w2_unloading_exit_row():
    # W2-unloading -> W3-unloading reverse exit: the chassis touches the road mesh
    # while all six wheels stand on the two approved W2 unloading buffer ramps.
    return dict(collision="SM_Road_20",
                collision_normal=[1.0435628610139247e-06,-.0041631897911429405,-.9999913573265076],
                collision_penetration_m=-.024687571451067924,
                wheel_surfaces=["TrafficW2UnloadingCurbToHardstand","TrafficW2UnloadingCurbToHardstand",
                                "TrafficW2UnloadingRoadToCurb","TrafficW2UnloadingRoadToCurb",
                                "TrafficW2UnloadingRoadToCurb","TrafficW2UnloadingRoadToCurb"],
                wheel_clearances_m=[-.012633056640625,-.016246337890625,.0406988525390625,
                                    .05444091796875,-.0170098876953125,-.0307513427734375],
                body_hits=[None,None,None], parked_vehicle_overlaps=[None,None,None])


W2_GROUND = {"TrafficW2UnloadingCurbToHardstand","TrafficW2UnloadingRoadToCurb"}


def test_road_mesh_under_chassis_is_ground_only_while_all_wheels_are_known():
    row=w2_unloading_exit_row()
    assert supported_ramp_contact(row,W2_GROUND,clearance_m=None)
    assert not supported_ramp_contact(row,W2_GROUND)


@pytest.mark.parametrize("change", [
    {"collision": "SM_Wall_156"},
    {"collision_normal": [.6,0,-.8]},
    {"collision_normal": [1,0,0]},
    {"collision_penetration_m": .09},
    {"collision_penetration_m": .03},
    {"wheel_surfaces": ["SM_Fence_1","TrafficW2UnloadingRoadToCurb","TrafficW2UnloadingRoadToCurb",
                        "TrafficW2UnloadingRoadToCurb","TrafficW2UnloadingRoadToCurb",
                        "TrafficW2UnloadingRoadToCurb"]},
    {"wheel_surfaces": []},
    {"body_hits": ["SM_Road_20",None,None]},
    {"parked_vehicle_overlaps": ["Car_0_2",None,None]},
    {"wheel_clearances_m": [.2]*6},
    {"wheel_clearances_m": [None]*6},
])
def test_unverified_road_mesh_contact_is_still_rejected(change):
    row=w2_unloading_exit_row()
    row.update(change)
    assert not supported_ramp_contact(row,W2_GROUND,clearance_m=None)

def q9_buffer_bevel_contact():
    return dict(collision='SM_Road_51',collision_normal=[.481256634,-.000002891,-.876579702],
        collision_penetration_m=.000088472,
        wheel_surfaces=['SM_Road_51','TrafficWaitingQ9Q10RoadToCurb','SM_Road_51','SM_Road_54','TrafficWaitingQ9Q10RoadToCurb','SM_Road_51'],
        wheel_clearances_m=[-.0492645,.0105591,-.0214795,-.07336,.0065753,-.0622009],
        body_hits=[None]*3,parked_vehicle_overlaps=[None]*3)

Q9_GROUND={'SM_Road_51','SM_Road_54','TrafficWaitingQ9Q10RoadToCurb'}

def test_observed_q9_buffer_bevel_is_project_tire_support_only():
    row=q9_buffer_bevel_contact()
    assert supported_ramp_contact(row,Q9_GROUND,clearance_m=None)
    assert not supported_ramp_contact(row,Q9_GROUND)

@pytest.mark.parametrize('change',[
    {'collision_normal':[.6,0,-.8]}, {'collision_normal':[1,0,0]},
    {'collision_penetration_m':.002}, {'wheel_clearances_m':[.13]*6},
    {'wheel_clearances_m':[None]*6}, {'wheel_surfaces':['SM_Road_51']*6},
    {'body_hits':['SM_Road_51',None,None]}, {'parked_vehicle_overlaps':['other',None,None]},
    {'collision':'SM_Wall_1'}])
def test_buffer_bevel_exception_does_not_accept_obstacles_or_unverified_support(change):
    row=q9_buffer_bevel_contact();row.update(change)
    assert not supported_ramp_contact(row,Q9_GROUND,clearance_m=None)

def q12_anonymous_support():
    return dict(collision='',collision_normal=[-.013893523,0,-.9999035],collision_penetration_m=.009036469,
        wheel_surfaces=['TrafficWaitingQ11Q12RoadToCurb']*2+['TrafficWaitingQ11Q12CurbToHardstand']*4,
        wheel_clearances_m=[.0361456,.0253003,.0515845,.0796136,.0282446,.0002155],
        body_hits=[None]*3,parked_vehicle_overlaps=[None]*3)

Q12_GROUND={'TrafficWaitingQ11Q12RoadToCurb','TrafficWaitingQ11Q12CurbToHardstand'}

def test_nameless_vertical_contact_requires_six_approved_buffer_supports():
    row=q12_anonymous_support()
    assert supported_ramp_contact(row,Q12_GROUND,clearance_m=None)
    assert not supported_ramp_contact(row,Q12_GROUND)

@pytest.mark.parametrize('change',[
    {'collision':'SM_Wall_1'}, {'collision':None}, {'collision_normal':[.3,0,-.95]},
    {'collision_penetration_m':.011}, {'wheel_surfaces':['SM_Road_51']*6},
    {'wheel_clearances_m':[None]*6}, {'wheel_clearances_m':[.13]*6},
    {'body_hits':['unknown',None,None]}, {'parked_vehicle_overlaps':['other',None,None]}])
def test_nameless_contact_is_rejected_without_complete_vertical_support(change):
    row=q12_anonymous_support();row.update(change)
    assert not supported_ramp_contact(row,Q12_GROUND|{'SM_Road_51'},clearance_m=None)


def test_buffer_support_accepts_submillimetre_signed_native_depth():
    row=q9_buffer_bevel_contact();row['collision_penetration_m']=-.000955245
    surfaces=set(row['wheel_surfaces'])
    assert supported_ramp_contact(row,surfaces,clearance_m=None)
    assert not supported_ramp_contact(row,surfaces,clearance_m=.12)
    row['collision_penetration_m']=-.01
    assert not supported_ramp_contact(row,surfaces,clearance_m=None)


def test_q9_signed_separation_is_ground_support_without_body_contact():
    row=q9_buffer_bevel_contact()
    row.update(collision_normal=[.45587736,0,-.89004266],collision_penetration_m=-.0065075,
        wheel_clearances_m=[-.08586,.00089,.01565,-.06136,.00029,-.04109])
    assert supported_ramp_contact(row,Q9_GROUND,clearance_m=None)
    assert not supported_ramp_contact(row,Q9_GROUND)
    row['body_hits'][0]='SM_Road_51'
    assert not supported_ramp_contact(row,Q9_GROUND,clearance_m=None)


def q36_road_buffer_observation():
    return dict(collision='SM_Road_55',collision_normal=[.5088765,-.000002584,-.8608395],
        collision_penetration_m=.00003413796,
        wheel_clearances_m=[-.0773621,-.0306567,.0350501,.0547925,.0047095,-.0402454],
        wheel_surfaces=['SM_Road_55']*2+['TrafficInternalWaitingRamp_Q36_plate',
            'TrafficInternalWaitingRamp_Q36_-1','TrafficInternalWaitingRamp_Q36_-1',
            'TrafficInternalWaitingRamp_Q36_plate'],body_hits=[None]*3,parked_vehicle_overlaps=[None]*3)


def test_round37_q36_shallow_wheel_support_keeps_strict_acceptance():
    row=q36_road_buffer_observation();surfaces=set(row['wheel_surfaces'])
    assert supported_ramp_contact(row,surfaces,clearance_m=None)
    assert not supported_ramp_contact(row,surfaces,clearance_m=.12)


@pytest.mark.parametrize('change',[
    {'body_hits':['SM_Fence_1',None,None]},
    {'parked_vehicle_overlaps':[None,'car',None]},
    {'collision_normal':[.6,0,-.8]},
    {'collision_penetration_m':.00101},
    {'wheel_clearances_m':[.121]*6},
    {'wheel_surfaces':['SM_Fence_1']*6},
])
def test_q36_buffer_support_exception_does_not_hide_obstacles(change):
    row=q36_road_buffer_observation();surfaces=set(row['wheel_surfaces']);row.update(change)
    assert not supported_ramp_contact(row,surfaces,clearance_m=None)
