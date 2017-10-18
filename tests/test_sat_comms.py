from Vehicle_class import Vehicle
import pytest
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
from Base_stn_class import *
from Acc_channel_class import *

def test_uw_condition():
    config = sim_config('tests/config/sim_config.csv')
    AUV_A = Vehicle(config, 0, 3, 10, 10, -10, 0)

    base = Base_Station(3)

    AUV_A.sat_up(base, 1)

    # Assert nothing was uploaded due to not being on surface
    assert base.log[0].x == []

def test_upload():
    config = sim_config('tests/config/sim_config.csv')
    AUV_A = Vehicle(config, 0, 3, 10, 10, 0, 0)
    AUV_B = Vehicle(config, 1, 3, 499, 0, 0, 0)
    AUV_C = Vehicle(config, 2, 3, 998, 0, 0, 0)

    base = Base_Station(3)

    AUV_A.sat_up(base, 1)

    assert (base.log[0].lon[-1] == AUV_A.lon and
            base.log[0].lat[-1] == AUV_A.lat and
            base.log[0].z[-1] == AUV_A.z)

def test_download():
    config = sim_config('tests/config/sim_config.csv')

    AUV_A = Vehicle(config, 0, 3, 10, 10, 0, 0)
    AUV_B = Vehicle(config, 1, 3, 499, 0, 0, 0)
    AUV_C = Vehicle(config, 2, 3, 998, 0, 0, 0)

    base = Base_Station(3)

    AUV_A.sat_up(base, 1)
    AUV_B.sat_up(base, 1)
    AUV_C.sat_up(base, 1)

    AUV_A.sat_down(config, base, 3, 1)

    assert (AUV_A.loc_pos[1][0] == AUV_B.lon and
            AUV_A.loc_pos[1][1] == AUV_B.lat and
            AUV_A.loc_pos[1][2] == AUV_B.z)

    assert(AUV_A.loc_pos[2][0] == AUV_C.lon and
            AUV_A.loc_pos[2][1] == AUV_C.lat and
            AUV_A.loc_pos[2][2] == AUV_C.z)

def test_time_stamps():
    config = sim_config('tests/config/sim_config.csv')
    pos = ((-1.3952, 50.8904),
           (-1.3970, 50.8910),
           (-1.4100, 50.8720))

    AUV_A = Vehicle(config, 0, 3, pos[0][0], pos[0][1], 0, 0)
    AUV_B = Vehicle(config, 1, 3, pos[1][0], pos[1][1], 0, 0)
    AUV_C = Vehicle(config, 2, 3, pos[2][0], pos[2][1], 0, 0)


    base = Base_Station(config.swarm_size)
    achannel = V2V_comms(config)
    elps_time = 0

    # Information exchanged via satellite
    AUV_A.sat_up(base, elps_time)
    AUV_B.sat_up(base, elps_time)
    AUV_C.sat_up(base, elps_time)
    AUV_A.sat_down(config, base, config.swarm_size, elps_time)
    AUV_B.sat_down(config, base, config.swarm_size, elps_time)
    AUV_C.sat_down(config, base, config.swarm_size, elps_time)

    for i in range(100):
        AUV_A.go(config)
        AUV_B.go(config)
        AUV_C.go(config)
        elps_time += 1

    # Condition AUVs must be more than 0.5m deep to transmit & receive acc
    AUV_A.z, AUV_B.z, AUV_C.z = -1,-1,-1
    AUV_B.send_acc_msg(achannel, elps_time, config, [AUV_A, AUV_B, AUV_C])
    # A should receive the acc data and update the timestamp
    AUV_A.receive_acc_msg(achannel)
    # C will not receieve the data as it is out of acc range
    AUV_C.receive_acc_msg(achannel)

    # A received the acoustic message therefore has a newer timestamp
    # for vehicle B
    assert AUV_A.time_stamps[1] == 100
    # C was out of range hence has the timestamp from initial sat comms
    assert AUV_C.time_stamps[1] == 0

    # B has not moved since acc comms hence...
    assert (AUV_A.loc_pos[1][0] == AUV_B.lon and
            AUV_A.loc_pos[1][1] == AUV_B.lat and
            AUV_A.loc_pos[1][2] == AUV_B.z)

    # Cs knowledge of B should be the same as that on the server
    # as it did not receive the acoustic message
    assert (AUV_C.loc_pos[1][0] == base.log[1].lon[-1] and
            AUV_C.loc_pos[1][1] == base.log[1].lat[-1] and
            AUV_C.loc_pos[1][2] == base.log[1].z[-1])

    # C should not know where B is currently
    assert (AUV_C.loc_pos[1][0] != AUV_B.lon and
            AUV_C.loc_pos[1][1] != AUV_B.lat and
            AUV_C.loc_pos[1][2] != AUV_B.z)
#
# def test_sat_loc_vehicles():
#     AUV_A = Vehicle(0, 3, 10, 10, 0, 0)
#     AUV_B = Vehicle(1, 3, 50, 50, 0, 0)
#     AUV_C = Vehicle(2, 3, 501, 501, 0, 0)
#
#     config = sim_config('tests/config/sim_config.csv')
#     base = Base_Station(config.swarm_size)
#     achannel = V2V_comms(config)
#     elps_time = 0
#     # Information exchanged via satellite
#     AUV_A.sat_up(base, elps_time)
#     AUV_B.sat_up(base, elps_time)
#     AUV_C.sat_up(base, elps_time)
#     AUV_A.sat_down(base, config.swarm_size, elps_time)
#     AUV_B.sat_down(base, config.swarm_size, elps_time)
#     AUV_C.sat_down(base, config.swarm_size, elps_time)
#
#     assert (AUV_A.loc_vehicles[0] == 0 and
#             AUV_A.loc_vehicles[1] == 1 and
#             AUV_A.loc_vehicles[2] == 1
#             )
#
#     elps_time = 50
#
#     # Information exchanged via satellite
#     AUV_A.sat_up(base, elps_time)
#     AUV_A.sat_down(base, config.swarm_size, elps_time)
#
#     # There has been no update from B or C since last 'surface'
#     # hence data rendered uncertain and B & C removed from loc_vehicles
#
#     assert (AUV_A.loc_vehicles[0] == 0 and
#             AUV_A.loc_vehicles[1] == 0 and
#             AUV_A.loc_vehicles[2] == 0
#             )
#
#     AUV_B.sat_up(base, elps_time)
#     AUV_B.sat_down(base, config.swarm_size, elps_time)
#
#     # There has been an update from A since last sat comm hence A kept
#     # in loc vehicles but no update from C means C is removed.
#
#     assert (AUV_B.loc_vehicles[0] == 1 and
#             AUV_B.loc_vehicles[1] == 0 and
#             AUV_B.loc_vehicles[2] == 0
#             )
#
# def test_sat_commd():
#     AUV_A = Vehicle(0, 3, 10, 10, 0, 0)
#
#     config = sim_config('tests/config/sim_config.csv')
#     base = Base_Station(config.swarm_size)
#     elps_time = 0
#
#     AUV_A.sat_comms(base, config, elps_time)
#     assert AUV_A.state == 3
#     assert AUV_A.sat_commd == 0
#
#     AUV_A.pitch = AUV_A.config.max_pitch
#     AUV_A.v = 0
#
#     AUV_A.sat_comms(base, config, elps_time)
#     assert AUV_A.state == 3
#     assert AUV_A.sat_commd == 1
#
#     elps_time = 181/config.time_step
#     AUV_A.waypoints = [[2,2,2],[4,4,4],[5,5,5]]
#     AUV_A.sat_comms(base, config, elps_time)
#     assert AUV_A.state == 0
#     assert AUV_A.sat_commd == 1
#
#     AUV_A.z = -1
#     AUV_A.sat_comms(base, config, elps_time)
#
#     assert AUV_A.sat_commd == 0