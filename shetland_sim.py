import sys
sys.path.insert(0,'../gen')
from Vehicle_class import Vehicle
import pandas as pd
import numpy as np
from Config_class import sim_config
from writing import *
from progress_bar import *

waypoints =pd.read_csv('../Results/data/Shetland_Comparison/real/waypoints.csv')
config = sim_config('config/sim_config.csv')
# Vehicle (config,i, Swarm_Size, start_lon, start_lat, start_z, start_yaw):)
AUV = Vehicle(config, 0, 1, -1.362773, 60.391747, 0, 0)
AUV.waypoints = [[waypoints.iloc[i].lon, waypoints.iloc[i].lat, waypoints.iloc[i].depth] for i in range(waypoints.shape[0])]
print(AUV.waypoints)
for elps_time in range(config.run_time):
    AUV.go(config, elps_time)
    update_progress(elps_time, config.run_time)
write_proof(AUV,config,'~/shetland_sim.csv')