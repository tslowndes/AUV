import sys
###################### USER FUNCTIONS ############################
sys.path.insert(0, "../gen")
from Base_stn_class import Base_Station
from Acc_channel_class import *
from progress_bar import *
from Vehicle_class import Vehicle
from writing import write_proof
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
    AUV = Vehicle(config, 0, config.swarm_size, start_loc[0], start_loc[1], 0, start_yaw)
    AUV.waypoints = [[50000, 0, 0]]
    v_demands = [1.0,1.5,0.75,0.25,0.0]

    for v in v_demands:
        AUV.v_demand = v
        for elps_time in range(100):
            AUV.go(config, elps_time)

    write_proof(AUV, config)
    print_time(1)

def prove_yaw_behaviour(config):
    print_time(0)
    start_yaw = 0
    AUV = Vehicle(config, 0, config.swarm_size, -1.3945, 50.8926, 0, start_yaw)
    AUV.waypoints = [[-1.3945, 50.8926, 0],
                    [-1.3965, 50.8926, 0],
                    [-1.3965, 50.8916, 0],
                    [-1.3945, 50.8916, 0]]

    for elps_time in range(config.run_time):
        AUV.go(config, elps_time)
        update_progress(elps_time, config.run_time)
    # Outputs results
    print('\nWriting Results')
    write_proof(AUV, config)
    print_time(1)

def prove_dive_behaviour(config):
    Base = Base_Station(config.swarm_size)
    AUV = Vehicle(config, 0, 1, -1.3945, 50.8926, 0, 0)
    AUV.waypoints = [[-1.3945, 50.8926, 0],
                    [-1.3955, 50.8926, -50],
                    [-1.3965, 50.8926, 0],
                    [-1.3965, 50.8921, -50],
                    [-1.3965, 50.8916, 0],
                    [-1.3955, 50.8916, -50],
                    [-1.3945, 50.8916, 0],
                     [-1.3945, 50.8916, -50],
                     [-1.3945, 50.8920, 0]]

    for elps_time in range(config.run_time):

            # Performs communication, sat if at surface, recieves acc if submerged
        AUV.sat_comms(Base, config, elps_time)

        AUV.time_checks(elps_time, config)
        AUV.go(config, elps_time)
    write_proof(AUV, config)

run_validation()