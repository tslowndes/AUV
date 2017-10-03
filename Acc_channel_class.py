import sys
sys.path.insert(0, "../gen")
import numpy as np
from latlon_util import find_dist3

class V2V_comms:
    def transmit_msg(self, vehicles, config):
        self.receive_msg = np.zeros(config.swarm_size)
        vehicle_pos = np.array([[AUV.lon, AUV.lat, AUV.z] for AUV in vehicles])
        dists = np.array([find_dist3(pos, self.msg.pos) for pos in vehicle_pos])
        self.receive_msg[np.where(dists < config.comms_range)[0]] = 1
        self.receive_msg[self.msg.ID] = 0

    def __init__(self, config):
        self.receive_msg = np.zeros(config.swarm_size)


class Msg:
    def __init__(self, ID, sent_time, v, pos, yaw, pitch):
        self.ID = ID
        self.sent_time = sent_time
        self.v = v
        self.pos = pos
        self.yaw = yaw
        self.pitch = pitch