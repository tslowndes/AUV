import sys
sys.path.insert(0, "../../AUV")
sys.path.insert(0, "../../gen")
from Base_stn_class import Base_Station
from Config_class import sim_config
from writing import write_proof


Base = Base_Station(1)
config = sim_config('config/sim_config_solo.csv')
AUV = Vehicle(0, 1, 0, 0, 0, 0)

AUV.waypoints = [[50,0,-50], [100, 0, -50], [100, 0, 0], [150, 0, -50], [200, 0, 0]]

for elps_time in range(5000):
    AUV.sat_comms(Base, config.swarm_size, elps_time)
    AUV.go(config, elps_time)

write_proof(AUV, config, fn = 'results/sat_delay_test.csv')