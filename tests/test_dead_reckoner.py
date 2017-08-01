import pytest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
import numpy as np

def test_pitch_yaw():
    init_pitch = np.random.randint(-40,40)
    init_yaw = np.random.randint(0, 360)
    init_x, init_y = 0,0
    if init_pitch < 0:
        init_z = -50
    else:
        init_z = 0
    # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(0, 1, init_x, init_y, init_z, init_yaw)
    config = sim_config('config/sim_config.csv')
    AUV.set_yaw(init_yaw)
    AUV.set_pitch(init_pitch)

    AUV.dead_reckoner(config.time_step)
    vxy = AUV.v * np.cos(np.radians(AUV.pitch))

    x = init_x + (vxy * np.cos(np.radians(AUV.yaw)) * config.time_step)
    y = init_y + (vxy * np.sin(np.radians(AUV.yaw)) * config.time_step)
    z = init_z + (-AUV.v * (np.sin(np.radians(AUV.pitch))) * config.time_step)

    assert abs(AUV.x - x) < 0.0000001
    assert abs(AUV.y - y) < 0.0000001
    assert abs(AUV.z - z) < 0.0000001