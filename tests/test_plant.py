from Vehicle_class import Vehicle
import pytest
import sys
sys.path.insert(0, "../../gen")
from Config_class import *

@pytest.mark.parametrize("yaw,yaw_demand,expected", [
    (330,10,1),
    (10,330,0),
    (180,359,1),
    (178,359,0),
    (180,360,1)
])

def test_yaw_logic(yaw, yaw_demand, expected):
    init_x, init_y, init_z, init_yaw = 0,0,0,0
    AUV = Vehicle(0, 1, init_x, init_y, init_z, init_yaw)
    config = sim_config('config/sim_config.csv')

    AUV.set_yaw(yaw)
    AUV.set_yaw_demand(yaw_demand)

    AUV.plant(config.time_step)

    if expected ==1:
        assert AUV.yaw > yaw
    elif expected == 0:
        assert AUV.yaw < yaw


@pytest.mark.parametrize("start_loc, wayp, expected", [
    ((0,0,-50), (50,50,0), 0),
    ((0, 0, 0), (50, 50, -50), 1),
    ((0,0,0), (50,50,0), 2)
])

def test_pitch(start_loc, wayp, expected):
    AUV = Vehicle(0, 1, start_loc[0], start_loc[1], start_loc[2], 0)
    config = sim_config('config/sim_config.csv')

    AUV.waypoints = [wayp]
    AUV.current_waypoint = 0

    AUV.move_to_waypoint()
    AUV.plant(config.time_step)

    if expected == 1:
        # assert AUV pitches nose down
        assert AUV.pitch > 0
    elif expected == 0:
        # assert AUV pitches nose up
        assert AUV.pitch < 0
    elif expected == 2:
        # assert AUV pitches nose up
        assert AUV.pitch == 0

@pytest.mark.parametrize("v,v_demand,expected", [
    (1,0.5,0),
    (0.5, 1, 1),
    (1,1.25,2),
    (0,0,2)
])


def test_v(v, v_demand, expected):
    AUV = Vehicle(0, 1, 0, 0, 0, 0)
    config = sim_config('config/sim_config.csv')

    AUV.set_v(v)
    AUV.set_v_demand(v_demand)
    AUV.plant(config.time_step)

    if expected == 1:
        # assert AUV pitches nose down
        assert AUV.v > v
    elif expected == 0:
        # assert AUV pitches nose up
        assert AUV.v < v
    elif expected == 2:
        # assert AUV pitches nose up
        assert AUV.v == v