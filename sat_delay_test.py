import sys
sys.path.insert(0, "../../AUV")
sys.path.insert(0, "../../gen")
from Base_stn_class import Base_Station
from Config_class import sim_config
from writing import write_proof
from Vehicle_class import Vehicle
import pandas as pd
import matplotlib.pyplot as plt

def main():
    Base = Base_Station(1)
    config = sim_config('config/sim_config.csv')
    AUV = Vehicle(0, 1, 0, 0, 0, 0)

    AUV.waypoints = [[50,0,-50], [100, 0, -50], [100, 0, 0], [150, 0, -50], [200, 0, 0]]

    for elps_time in range(5000):
        AUV.sat_comms(Base, config, elps_time)
        AUV.go(config, elps_time)

    write_proof(AUV, config, fn = 'results/sat_delay_test.csv')

    df = pd.read_csv('results/sat_delay_test.csv')
    plt.plot(df.index, df.z)
    plt.show()

if __name__=='__main__': main()