from Vehicle_class import Vehicle
import pytest
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
from Acc_channel_class import *
import numpy as np


def test_range():
    config = sim_config('tests/config/sim_config.csv')
# self, config,i, Swarm_Size, start_lon, start_lat, start_z, start_yaw):
    AUV_A = Vehicle(config, 0, 3, 1.958749, 56.404459, -10, 0)
    # AUV_B is in range of AUV_A with comms range 500
    AUV_B = Vehicle(config, 1, 3, 1.9644664200733279, 56.407622206929943, -10, 0)
    # AUV_C is out of range of AUV_A with comms range 500
    AUV_C = Vehicle(config, 2, 3, 1.9690823801841355, 56.410175447203088, -10, 0)


    achannel = V2V_comms(config)


    AUV_A.send_acc_msg(achannel, 1, config, [AUV_A, AUV_B, AUV_C])

    AUV_B.receive_acc_msg(achannel)
    AUV_C.receive_acc_msg(achannel)

    # assert B received the message
    assert AUV_B.loc_pos[0][0] == AUV_A.lon
    assert AUV_B.loc_pos[0][1] == AUV_A.lat
    assert AUV_B.loc_pos[0][2] == AUV_A.z

    # assert C did not receive the message
    assert AUV_C.loc_pos[0][0] != AUV_A.lon
    assert AUV_C.loc_pos[0][1] != AUV_A.lat
    assert AUV_C.loc_pos[0][2] != AUV_A.z

@pytest.mark.parametrize("loc_1, loc_2, loc_3, expected", [
    ((0, 0, -1), (0, 0, -50), (0, 0, -502), (0, 1, 0)),
    ((-1.3952, 50.8904, -1), (-1.3881,50.8904,-1), (-1.3874, 50.8904, -1), (0, 1, 0)),
    ((-1.3952, 50.8904, -1), (-1.3891, 50.8942, -50), (-1.3347, 50.9284, 500), (0, 0, 0)),
])

def test_transmit(loc_1, loc_2, loc_3, expected):
    config = sim_config('tests/config/sim_config.csv')

    AUV_A = Vehicle(config, 0, 3, loc_1[0], loc_1[1], loc_1[2], 0)
    AUV_B = Vehicle(config, 1, 3, loc_2[0], loc_2[1], loc_2[2], 0)
    AUV_C = Vehicle(config, 2, 3, loc_3[0], loc_3[1], loc_3[2], 0)

    achannel = V2V_comms(config)

    AUV_A.send_acc_msg(achannel, 5, config, [AUV_A, AUV_B, AUV_C])

    assert (achannel.receive_msg[0] == expected[0] and
            achannel.receive_msg[1] == expected[1] and
            achannel.receive_msg[2] == expected[2])

@pytest.mark.parametrize("msg_time, time_1, time_2, expected", [
    # B reject, C accept
    (51, 100, 50, (100, 51)),
    # B Accept, C accept
    (51, 50, 50, (51, 51)),
    # B reject, C reject
    (51, 52, 52, (52, 52)),
    # B accept, C reject
    (51, 50, 52, (51, 52)),
    # minus number?
    (50, -10, -10, (50,50))
])

def test_sent_time(msg_time, time_1, time_2, expected):
    # Examines the condition of accepting messages based on the message timestamp
    # and the timestamp of the corresponding data held by the AUV
    config = sim_config('tests/config/sim_config.csv')
    AUV_A = Vehicle(config, 0, 3, -1.3347, 50.9284, -1, 0)
    AUV_B = Vehicle(config, 1, 3, -1.3348, 50.9285, -1, 0)
    AUV_C = Vehicle(config, 2, 3, -1.3346, 50.9283, -1, 0)

    achannel = V2V_comms(config)

    AUV_B.time_stamps[0] = time_1
    AUV_C.time_stamps[0] = time_2

    AUV_A.send_acc_msg(achannel, msg_time, config, [AUV_A, AUV_B, AUV_C])
    AUV_B.receive_acc_msg(achannel)
    AUV_C.receive_acc_msg(achannel)

    assert AUV_B.time_stamps[0] == expected[0]
    assert AUV_C.time_stamps[0] == expected[1]

@pytest.mark.parametrize("comms_AUV", [
    0,
    1,
    2
])

def test_acc_loc_vehicles(comms_AUV):
    config = sim_config('tests/config/sim_config.csv')
    # B is in range of A and C
    # A is out of range of C
    AUV_A = Vehicle(config, 0, 3, -1.3952, 50.8904, -1, 0)
    AUV_B = Vehicle(config, 1, 3, -1.3912, 50.8929, -1, 0)
    AUV_C = Vehicle(config, 2, 3, -1.3871, 50.8954, -1, 0)

    achannel = V2V_comms(config)
    if comms_AUV == 0:
        AUV_A.send_acc_msg(achannel,0,config, [AUV_A, AUV_B, AUV_C])
    elif comms_AUV == 1:
        AUV_B.send_acc_msg(achannel, 0, config, [AUV_A, AUV_B, AUV_C])
    elif comms_AUV == 2:
        AUV_C.send_acc_msg(achannel,0,config, [AUV_A, AUV_B, AUV_C])

    AUV_A.receive_acc_msg(achannel)
    AUV_B.receive_acc_msg(achannel)
    AUV_C.receive_acc_msg(achannel)

    if comms_AUV == 0:
        # B is in range hence As index in Bs loc_vehicles = 1
        # C is out of range: As index in B.loc_vehicles = 0
        assert AUV_B.loc_vehicles[0] == 1
        assert AUV_C.loc_vehicles[0] == 0
    elif comms_AUV == 1:
        assert AUV_A.loc_vehicles[1] == 1
        assert AUV_C.loc_vehicles[1] == 1
    elif comms_AUV == 2:
        assert AUV_A.loc_vehicles[2] == 0
        assert AUV_B.loc_vehicles[2] == 1