import pytest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
from Base_stn_class import *
import numpy as np

@pytest.mark.parametrize("elps_time, state, expected", [
    # Just started dive, should stay diving
    (50, 0, 0),
    (601, 0, 1), #Reached 50% t_uw hence should be surfacing
    (1199, 1, 2), #Surpassed 90% t_uw and surfacing, should immediately surface
    (1000, 0, 1), #Surpassed 50% by a long way and somehow still diving?
    (1100, 0, 2), #Surpassed 90% t_uw and somehow still diving? go straight to immediate surface
    (1500, 1, 2),
    (1500, 2, 2),
    (1500, 0, 2)
])

def test_time_checks(elps_time, state, expected):
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(0, 1, 0, 0, -50, 0)
    config = sim_config('config/sim_config.csv')

    AUV.state = state

    #t_uw set to 1200
    AUV.time_checks(elps_time, config)

    assert AUV.state == expected

@pytest.mark.parametrize("z, state, expected", [
    (-1, 1, 1),
    (-0.1, 1, 0),
    (-1, 2, 2),
    (-0.1, 2, 0),
    # faulty sensor, depth !> 0, but > -0.5 hence still sat_comm & dive.
    (50, 2, 0)
])


def test_surfacing(z, state, expected):
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(0, 1, 0, 0, z, 0)
    config = sim_config('config/sim_config.csv')
    base = Base_Station(config.swarm_size)
    # AUV at 1m depth and surfacing
    AUV.state = state

    # Try to sat comm (this triggers state change to diving
    # should remain surfacing as deeper than conditional depth for sat (0.5m)
    AUV.sat_comms(base, config.swarm_size, 0)

    assert AUV.state == expected

def test_reached_waypoint():
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(0, 1, 50, 50,-10 , 0)
    config = sim_config('config/sim_config.csv')
    AUV.state = 0

    # AUV at 1m depth and surfacing
    AUV.waypoints = [[50,50,-20]]
    AUV.move_to_waypoint()

    assert AUV.state == 0

    AUV.z = -19
    AUV.move_to_waypoint()

    assert AUV.state == 1

# Expected - 0: accepts state, 1:errors
@pytest.mark.parametrize("curr_state, desired_state, z, expected", [
    (1, 0, -0.1, 0),
    (2, 0, -0.1, 0),
    (1, 0, -0.7, 1),
    (2, 0, -0.7, 1),
    (2, 1, -10, 1),
    (2, 1, -4000, 1),
    (2, 1, -0.1, 1)
])

def test_set_state(curr_state, desired_state, z, expected):
    AUV = Vehicle(0, 1, 50, 50, -10, 0)
    AUV.state = curr_state
    AUV.z = z

    if expected == 1:
        with pytest.raises(Exception) as e_info:
            AUV.set_state(desired_state)
    else:
        AUV.set_state(desired_state)
        assert AUV.state == desired_state
