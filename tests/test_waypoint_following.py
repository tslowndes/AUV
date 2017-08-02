import pytest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
import numpy as np

@pytest.mark.parametrize("target, expected", [
    ((100, 0, 0), 0),
    ((0, 100, 0), 90),
    ((0, -100, 0), 270),
    ((-100, 0, 0), 180),
    ((50, 50, 0), 45),
    ((-50, 50, 0), 135),
    ((-50, -50, 0), 225),
    ((50, -50, 0), 315),
])

def test_yaw_demand(target, expected):
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(0, 1, 0, 0, 0, 0)
    config = sim_config('config/sim_config.csv')
    AUV.waypoints = [target]
    AUV.move_to_waypoint()
    assert AUV.yaw_demand == expected

@pytest.mark.parametrize("start, target, expected", [
    ((0, 0, 0), (100, 0, 0), 0),
    ((0, 0, 0), (0, 400, -50), 7.125016349),
    ((0, 0, 0), (0, 0, -50), 40),
    ((0, 0, -50), (0, 0, 0), -40),
    ((0, 0, -50), (0, 400, 0), -7.125016349),
])

def test_pitch_demand(start, target, expected):
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(0, 1, start[0], start[1], start[2], 0)
    config = sim_config('config/sim_config.csv')
    AUV.waypoints = [target]
    AUV.move_to_waypoint()
    assert abs(AUV.pitch_demand - expected) < 0.0000001
