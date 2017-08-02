from Vehicle_class import Vehicle
import pytest
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
from Acc_channel_class import *
import numpy as np


def test_range():

    AUV_A = Vehicle(0, 3, 10, 10, -10, 0)
    AUV_B = Vehicle(1, 3, 499, 0, -10, 0)
    AUV_C = Vehicle(2, 3, 998, 0, -10, 0)

    config = sim_config('tests/config/sim_config.csv')
    achannel = V2V_comms(config)


    AUV_A.send_acc_msg(achannel, 1, config, [AUV_A, AUV_B, AUV_C])

    AUV_B.receive_acc_msg(achannel)
    AUV_C.receive_acc_msg(achannel)

    # assert B received the message
    assert AUV_B.loc_pos[0][0] == AUV_A.x
    assert AUV_B.loc_pos[0][1] == AUV_A.y
    assert AUV_B.loc_pos[0][2] == AUV_A.z

    # assert C did not receive the message
    assert AUV_C.loc_pos[0][0] != AUV_A.x
    assert AUV_C.loc_pos[0][1] != AUV_A.y
    assert AUV_C.loc_pos[0][2] != AUV_A.z

@pytest.mark.parametrize("loc_1, loc_2, loc_3, expected", [
    ((0, 0, -1), (0, 0, -50), (0, 0, -502), (0, 1, 0)),
    ((0, 0, -1), (0, 499, -1), (0, 501, -1), (0, 1, 0)),
    ((0, 0, -1), (50, 490, -50), (500, 500, 500), (0, 1, 0)),
    ((0, 0, -1), (500, 490, -50), (500, 500, 500), (0, 0, 0)),
])

def test_transmit(loc_1, loc_2, loc_3, expected):
    AUV_A = Vehicle(0, 3, loc_1[0], loc_1[1], loc_1[2], 0)
    AUV_B = Vehicle(1, 3, loc_2[0], loc_2[1], loc_2[2], 0)
    AUV_C = Vehicle(2, 3, loc_3[0], loc_3[1], loc_3[2], 0)

    config = sim_config('tests/config/sim_config.csv')
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

def test_sent_time(msg_time, time_1, time_2):
    AUV_A = Vehicle(0, 3, 1, 1, -1, 0)
    AUV_B = Vehicle(1, 3, 50, 50, -1, 0)
    AUV_C = Vehicle(2, 3, 30, 50, -1, 0)

    config = sim_config('tests/config/sim_config.csv')
    achannel = V2V_comms(config)

    # Because the timestamp on the message is 51, it will not be accepted by
    # AUV_B but will be by AUV_C
    AUV_B.time_stamps[0] = time_1
    AUV_C.time_stamps[0] = time_2

    AUV_A.send_acc_msg(achannel, msg_time, config, [AUV_A, AUV_B, AUV_C])
    AUV_B.receive_acc_msg(achannel)
    AUV_C.receive_acc_msg(achannel)

    # B Rejected and kept more recent data
    assert AUV_B.time_stamps[0] == expected[0]
    # C Accepted more up to date data
    assert AUV_C.time_stamps[0] == expected[1]