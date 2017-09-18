import pytest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
import numpy as np


@pytest.mark.parametrize("z, yaw, pitch, expected", [
    (0, 0, 0, (0.25, 0, 0)),
    (0, 45, 0, (0.35355339059327*0.5, 0.35355339059327*0.5, 0)),
    (0, 90, 0, (0, 0.25, 0)),
    (0, 180, 0, (-0.25, 0, 0)),
    (0, 270, 0, (0, -0.25, 0)),
    (0, 40, 25, (0.17356801, 0.145640855, -0.105654565)),
    # 50 is outside max pitch (40)
    (0, 360, 50, (0.3830222216*0.5, 0, -0.32139380484*0.5)),
    # -10 should be considered as 350degs
    (0, -10, 0, (0.2462019382531, -0.043412044416732597, 0)),
    (-50, 170, -20, (-0.462708289*0.5, 0.081587956*0.5, -50-(-0.17101007166*0.5))),
    (-50, 250, -50, (-0.1310013151146924*0.5, -0.35992315519647711*0.5, -49.839303097578366)),
    (0, 0,-40,(0.191511111,0,0))
])


def test_pitch_yaw(z, yaw, pitch, expected):
    init_x, init_y = 0,0
    config = sim_config('config/sim_config.csv')
    # (self, config, i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(config, 0, 1, init_x, init_y, z, yaw)
    config = sim_config('config/sim_config.csv')
    AUV.set_yaw(yaw)
    AUV.set_pitch(pitch)

    AUV.dead_reckoner(config.time_step)

    x = expected[0]
    y = expected[1]
    z = expected[2]

    assert abs(AUV.x - x) < 0.0000001
    assert abs(AUV.y - y) < 0.0000001
    assert abs(AUV.z - z) < 0.0000001