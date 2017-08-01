from Vehicle_class import Vehicle
import pytest
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
from Acc_channel_class import *
#
# @pytest.mark.parametrize("yaw,yaw_demand,expected", [
#
# ])
#
# start_locs = [[0, 0, 0], [550,550,0]]
#
# def test_range(start_locs):
#
#     AUV = Vehicle(0, 1, init_x, init_y, init_z, init_yaw)
#     config = sim_config('config/sim_config.csv')
#
#     AUV.set_yaw(yaw)
#     AUV.set_yaw_demand(yaw_demand)
#
#     AUV.plant(config.time_step)
#
#     if expected ==1:
#         assert AUV.yaw > yaw
#     elif expected == 0:
#         assert AUV.yaw < yaw