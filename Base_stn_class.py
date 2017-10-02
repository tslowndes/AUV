import sys
sys.path.insert(0, '../gen')
from log import *

class Base_Station:
    def __init__(self, swarm_size):
        self.log = []
        self.time_stamps = []
        for i in range(swarm_size):
            self.time_stamps.append(0)
            self.log.append(Log())
