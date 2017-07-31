import numpy as np
from numba import jitclass

class V2V_comms:
    def transmit_msg(self, vehicles, config):
        self.receive_msg = np.zeros(config.swarm_size)
        vehicle_pos = np.array([[AUV.x, AUV.y, AUV.z] for AUV in vehicles])
        dists = np.sqrt(np.sum((vehicle_pos - self.msg.pos)**2, axis = 1))
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