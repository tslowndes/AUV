import pytest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
import numpy as np


@pytest.mark.parametrize("init_lon, init_lat, v, yaw, pitch, expected", [
    (1.958749, 56.404459, 1, 180, 0, (1.958749, 56.404454508397521, 0)),
    (1.958749, 56.404459, 0.5, 180, 0, (1.958749, 56.404456754198755, 0)),
    (1.958749, 56.404459, 10, 180, 0, (1.958749, 56.404454508397521, 0)),
    (1.958749, 56.404459, 0, 180, 0, (1.958749, 56.404459,0)),
    (1.958749, 56.404459, 1, 90, 0, (1.9587571174508021, 56.404458999999733,0)),
    (1.958749, 56.404459, 1, 270, 0, (1.9587408825491983, 56.404458999999733,0)),
    (1.958749, 56.404459, 1, 0, 0, (1.958749, 56.404463491602485,0)),
    (1.958749, 56.404459, 1, 360, 0, (1.958749, 56.404463491602485,0)),
    (1.958749, 56.404459, 1, 45, 0, (1.9587547399049874, 56.404462176042436,0)),
    (1.958749, 56.404459, 1, 45, 10, (1.9587546527029256,56.404462127791227,-0.086824088833465166)),
    (1.958749, 56.404459, 1, 45, 20, (1.9587543937463336,56.404461984503655,-0.17101007166283436)),
    (1.958749, 56.404459, 1, 45, 40, (1.9587533970222337,56.404461432989685,-0.32139380484326963)),
    (1.958749, 56.404459, 1, 45, 60, (1.9587533970222337,56.404461432989685,-0.32139380484326963))
])

# self, config,i, Swarm_Size, start_lon, start_lat, start_z, start_yaw):
def test_pitch_yaw(init_lon, init_lat, v, yaw, pitch, expected):
    config = sim_config('config/sim_config.csv')
    # (self, config, i, Swarm_Size, start_x, start_y, start_z, start_yaw)
    AUV = Vehicle(config, 0, 1, init_lon, init_lat, 0, yaw)
    config = sim_config('config/sim_config.csv')
    AUV.set_yaw(yaw)
    AUV.set_pitch(pitch)
    AUV.set_v(v)

    AUV.dead_reckoner(config)

    assert abs(AUV.lon - expected[0]) < 0.0000001
    assert abs(AUV.lat - expected[1]) < 0.0000001
    assert abs(AUV.z - expected[2]) < 0.0000001