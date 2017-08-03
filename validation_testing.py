import sys
###################### USER FUNCTIONS ############################
sys.path.insert(0, "../gen")
from Base_stn_class import Base_Station
from Acc_channel_class import *
from progress_bar import *
from Vehicle_class import Vehicle
from writing import write_proof
from dist import dist
from Config_class import *
import os
from shutil import copyfile

def run_validation():
    configs = []
    config_nos = [1,2,3]
    for i in config_nos:
        configs.append(sim_config('validation/vld_config_%03i.csv' % i))

    for config in configs:
        print('Running validation %i' % config.no)
        directory = 'validation'
        if os.path.exists(directory) is False:
            os.makedirs(directory)
        copyfile('config/vehicle_config.csv', directory + '/vehicle_config_%03i.csv' % config.no)
        validation_tests(config)

def validation_tests(config):
    if config.sim_sub_type == 1:
        prove_dive_behaviour(config)
    elif config.sim_sub_type == 2:
        prove_yaw_behaviour(config)
    elif config.sim_sub_type == 3:
        prove_vel_behaviour(config)

def prove_vel_behaviour(config):
    print_time(0)
    start_loc = [0, 0]
    start_yaw = 0
    AUV = Vehicle(0, config.swarm_size, start_loc[0], start_loc[1], 0, start_yaw)
    AUV.waypoints = [[50000, 0, 0]]
    v_demands = [1.0,1.5,0.75,0.25,0.0]

    for v in v_demands:
        AUV.v_demand = v
        for elps_time in range(100):
            AUV.go(config)

    write_proof(AUV, config)
    print_time(1)

def prove_yaw_behaviour(config):
    print_time(0)
    start_loc = [0,0]
    start_yaw = 0
    AUV = Vehicle(0, config.swarm_size, start_loc[0], start_loc[1], 0, start_yaw)
    AUV.waypoints = [[50,0,0],[50,50,0],[0,50,0],[0,0,0]]
    for elps_time in range(config.run_time):
        AUV.go(config)
        update_progress(elps_time, config.run_time)
    # Outputs results
    print('\nWriting Results')
    write_proof(AUV, config)
    print_time(1)

def prove_dive_behaviour(config):
    # INITALISATION
    comms_AUV = 0
    np.random.seed(42)
    start_locs = np.array([np.random.randint(50, size=config.swarm_size), np.random.randint(50, size=config.swarm_size)])
    start_yaws = np.random.randint(360, size=config.swarm_size)
    Base = Base_Station(config.swarm_size)
    Acc_comms = V2V_comms(config)
    Swarm = [Vehicle(0, config.swarm_size, start_locs[0][0], start_locs[1][0], 0, start_yaws[0])]
    print_time(0)

    # Initial communication step to populate data on subs
    for AUV in Swarm:
        AUV.sat_up(Base, 0)
    for AUV in Swarm:
        AUV.sat_down(Base, config.swarm_size, 0)
    check = 0
    # MAIN LOOP
    for elps_time in range(config.run_time):
        Swarm[comms_AUV].send_acc_msg(Acc_comms, elps_time, config, Swarm)
        for AUV in Swarm:
            # Performs communication, sat if at surface, recieves acc if submerged
            AUV.sat_comms(Base, config.swarm_size, elps_time)
            AUV.receive_acc_msg(Acc_comms)

        for AUV in Swarm:
            target = []
            ### Sets initial waypoint
            if elps_time == 0:
                target = [AUV.x + 50, AUV.y + 50]
            else:
                if AUV.state == 0:
                    dist_targ = dist([AUV.x, AUV.y, AUV.z], AUV.waypoints[AUV.current_waypoint], 3)
                elif AUV.state == 1:
                    dist_targ = dist([AUV.x, AUV.y, AUV.z], AUV.waypoints[AUV.current_waypoint], 3)
                ### Prevents waypoint being reached to demonstrate time limit underwater
                if dist_targ < 10 and elps_time < 1200*0.5:
                    target = [AUV.waypoints[AUV.current_waypoint][0] + 50, AUV.waypoints[AUV.current_waypoint][1] + 50]
                ### On the second surface (2 at start and 1 after surfacing from first dive = 3 sat comms)
                if len(AUV.log.sat_time_stamps) == 3 and AUV.state == 0 and check == 0:
                    # Set a reachable waypoint at depth
                    check = 1
                    target = [AUV.x + 50, AUV.y + 50]

            AUV.time_checks(elps_time, config)
            if target != []:
                AUV.set_waypoint(target, config)
            AUV.go(config)

        update_progress(elps_time, config.run_time)

    # Outputs results
    print('\nWriting Results')
    write_proof(Swarm[0], config)
    print_time(1)

run_validation()