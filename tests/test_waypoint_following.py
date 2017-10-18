import pytest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
import numpy as np

@pytest.mark.parametrize("target, expected", [
    ((-1.3952, 50.9000, 0), 0),
    ((-1.3838073279527769, 50.890399445600529, 0), 90),
    ((-1.4065926720472228, 50.890399445600529, 0), 270),
    ((-1.3952, 50.8000, 0), 180),
    ((-1.3871432853155921, 50.895481390874806, 0), 45),
    ((-1.3871450430656211, 50.885318054725715, 0), 135),
    ((-1.4032549569343786, 50.885318054725715, 0), 225),
    ((-1.4032567146844079, 50.895481390874806, 0), 315),
])

def test_yaw_demand(target, expected):
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    config = sim_config('config/sim_config.csv')
    AUV = Vehicle(config, 0, 1, -1.3952, 50.8904, 0, 0)
    AUV.waypoints = [target]
    elps_time = 0
    AUV.move_to_waypoint(elps_time, config)
    assert abs(AUV.yaw_demand - expected) < 0.001

@pytest.mark.parametrize("start, target, expected", [
    ((-1.3952, 50.8904, 0), (-1.3937759159829737, 50.890399991337503, 0), 0),
    ((-1.3952, 50.8904, 0), (-1.3895036639424887, 50.890399861400127, -50), 7.125016349),
    ((-1.3952, 50.8904, 0), (-1.3952, 50.8904, -50), 40),
    ((-1.3952, 50.8904, -50), (-1.3952, 50.8904, 0), -40),
    ((-1.3952, 50.8904, -50), (-1.3895036639424887, 50.890399861400127, 0), -7.125016349),
])

def test_pitch_demand(start, target, expected):
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    config = sim_config('config/sim_config.csv')
    AUV = Vehicle(config, 0, 1, start[0], start[1], start[2], 0)
    AUV.waypoints = [target]
    AUV.move_to_waypoint(0, config)
    assert abs(AUV.pitch_demand - expected) < 0.0000001
#
# @pytest.mark.parametrize("state, start, target, expected", [
#     (0, (0, 0, 0), [100, 100], (100, 100, -50)),
#     (1, (0, 0, -50), [100, 100], (100, 100, 0)),
#     (0, (0, 0, 0), [200, 200], (141.421356237, 141.421356237, -50)),
#     (1, (0, 0, 0), [200, 200], (141.421356237, 141.421356237, 0)),
#     (2, (0, 0, 0), [200, 200], (0, 0, 0)),
# ])
#
# def test_waypoint_setting(state, start, target, expected):
#     # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
#     AUV = Vehicle(0, 1, start[0], start[1], start[2], 0)
#     config = sim_config('config/sim_config.csv')
#     AUV.state = state
#     AUV.set_waypoint(target, config)
#
#     assert (abs(AUV.waypoints[0][0] - expected[0]) < 0.0000001 and
#             abs(AUV.waypoints[0][1] - expected[1]) < 0.0000001 and
#             abs(AUV.waypoints[0][2] - expected[2]) < 0.0000001)
#
# def test_waypoint_sequence():
#     AUV = Vehicle(0, 1, 50, 50, -49, 0)
#     config = sim_config('config/sim_config.csv')
#     AUV.waypoints = [[50,50,-50],[100,100,0]]
#     AUV.move_to_waypoint()
#     assert AUV.current_waypoint == 1
