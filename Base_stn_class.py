import sys
sys.path.insert(0, '/noc/users/tsl1g12/gen')
sys.path.insert(0, 'C:/users/tsl1g12/desktop/phd_sim/gen')
from log import *

class Base_Station:
    def __init__(self, swarm_size):
        self.log = []
        self.time_stamps = []
        for i in range(swarm_size):
            self.time_stamps.append(0)
            self.log.append(Log())
