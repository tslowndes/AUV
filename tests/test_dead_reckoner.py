import unittest
from Vehicle_class import Vehicle
import sys
sys.path.insert(0, "../../gen")
from Config_class import *
import numpy as np

def repeat(times):
    def repeatHelper(f):
        def callHelper(*args):
            for i in range(0, times):
                f(*args)

        return callHelper

    return repeatHelper

class test_pitch_0_yaw_0(unittest.TestCase):
    def setUp(self):
        # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
        self.AUV = Vehicle(0, 1, 0, 0, 0, 0)
        self.config = sim_config('config/sim_config.csv')

    def test_go(self):
        print('Testing yaw = %i degrees and pitch = %i' % (self.AUV.yaw,self.AUV.pitch))
        self.AUV.dead_reckoner(self.config.time_step)
        assert self.AUV.x == 0.25
        assert self.AUV.y == 0

    def tearDown(self):
        del self.AUV

class test_pitch_0_yaw_90(unittest.TestCase):
    def setUp(self):
        self.AUV = Vehicle(0, 1, 0, 0, 0, 0)
        self.config = sim_config('config/sim_config.csv')
        self.AUV.set_yaw(90)

    def test_go(self):
        print('Testing yaw = %i degrees and pitch = %i' % (self.AUV.yaw,self.AUV.pitch))
        self.AUV.dead_reckoner(self.config.time_step)
        # Due to precision of pi x is not exactly 0
        self.assertAlmostEqual(self.AUV.x, 0)
        assert self.AUV.y == 0.25

    def tearDown(self):
        del self.AUV

class test_pitch_0_yaw_180(unittest.TestCase):
    def setUp(self):
        # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
        self.AUV = Vehicle(0, 1, 0, 0, 0, 0)
        self.config = sim_config('config/sim_config.csv')
        self.AUV.set_yaw(180)

    def test_go(self):
        print('Testing yaw = %i degrees and pitch = %i' % (self.AUV.yaw,self.AUV.pitch))
        self.AUV.dead_reckoner(self.config.time_step)
        assert self.AUV.x == -0.25
        self.assertAlmostEqual(self.AUV.y, 0)

    def tearDown(self):
        del self.AUV

class test_pitch_0_yaw_270(unittest.TestCase):
    def setUp(self):
        # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
        self.AUV = Vehicle(0, 1, 0, 0, 0, 0)
        self.config = sim_config('config/sim_config.csv')
        self.AUV.set_yaw(270)

    def test_go(self):
        print('Testing yaw = %i degrees and pitch = %i' % (self.AUV.yaw,self.AUV.pitch))
        self.AUV.dead_reckoner(self.config.time_step)
        self.assertAlmostEqual(self.AUV.x, 0)
        assert self.AUV.y == -0.25

    def tearDown(self):
        del self.AUV

class test_pitch_yaw(unittest.TestCase):
    def setUp(self):
        self.init_pitch = np.random.randint(-40,40)
        self.init_yaw = np.random.randint(0, 360)
        self.init_x, self.init_y = 0,0
        if self.init_pitch < 0:
            self.init_z = -50
        else:
            self.init_z = 0

        # (self,i, Swarm_Size, start_x, start_y, start_z, start_yaw)
        self.AUV = Vehicle(0, 1, self.init_x, self.init_y, self.init_z, self.init_yaw)
        self.config = sim_config('config/sim_config.csv')
        self.AUV.set_yaw(self.init_yaw)
        self.AUV.set_pitch(self.init_pitch)

    def test_go(self):
        print('Testing yaw = %i degrees and pitch = %i' % (self.AUV.yaw, self.AUV.pitch))
        self.AUV.dead_reckoner(self.config.time_step)
        vxy = self.AUV.v * np.cos(np.radians(self.AUV.pitch))
        self.assertAlmostEqual(self.AUV.x, self.init_x + (vxy * np.cos(np.radians(self.AUV.yaw)) * self.config.time_step))
        self.assertAlmostEqual(self.AUV.y, self.init_y + (vxy * np.sin(np.radians(self.AUV.yaw)) * self.config.time_step))
        z = self.init_z + (-self.AUV.v * (np.sin(np.radians(self.AUV.pitch))) * self.config.time_step)
        self.assertAlmostEqual(self.AUV.z, z)

    def tearDown(self):
        del self.AUV
