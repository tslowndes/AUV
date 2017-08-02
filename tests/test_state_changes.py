import pytest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
import numpy as np

def test_time_checks(elps_time, state):
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(0, 1, 0, 0, -50, 0)
    config = sim_config('config/sim_config.csv')

    AUV.state = state

    #t_uw set to 1200
    AUV.time_checks(elps_time, config)

