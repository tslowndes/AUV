import sys
sys.path.insert(0, "../gen")
from math import sqrt, atan, exp, cos, sin, radians, degrees, ceil, floor, pi, isnan
from numpy import abs
import numpy as np

from find_dir import find_dir
from log import Log
from dist import dist
from Config_class import *
from Acc_channel_class import *


class Vehicle:
    def send_acc_msg(self, Acc_comms, elps_time, config, Swarm):
        if self.z < -0.5 and config.sat_andor_acc == 1:
            Acc_comms.msg = Msg(self.ID, elps_time, self.v, [self.x, self.y, self.z] , self.yaw, self.pitch)
            Acc_comms.transmit_msg(Swarm, config)

    def receive_acc_msg(self, Acc_comms):
        if Acc_comms.receive_msg[self.ID] == 1:
            msg = Acc_comms.msg
            self.loc_vehicles[msg.ID] = 1
            if msg.sent_time > self.time_stamps[msg.ID]: #[1]
                self.time_stamps[msg.ID] = msg.sent_time
                self.loc_v[msg.ID] = msg.v
                self.set_loc_pos(msg.ID, msg.pos)
                self.loc_yaw[msg.ID] = msg.yaw
                self.loc_pitch[msg.ID] = msg.pitch

    def sat_comms(self, base, swarm_size, elps_time):
        if self.z > -0.5 and self.sat_commd == 0:
            self.sat_up(base, elps_time)
            self.sat_down(base, swarm_size, elps_time)
            self.set_state(0)
            self.t_state_change = elps_time
            self.sat_commd = 1

        elif self.z < -0.5 and self.sat_commd == 1:
            self.sat_commd = 0

    def sat_up(self, base, elps_time):
        if self.z > -0.5:
            # Upload
            base.log[self.ID].x.append(self.x)
            base.log[self.ID].y.append(self.y)
            base.log[self.ID].z.append(self.z)
            base.log[self.ID].v.append(self.v)
            base.time_stamps[self.ID] = elps_time
            self.log.sat_time_stamps.append(elps_time)

    def sat_down(self, base, swarm_size, elps_time):
        if self.z > -0.5:
            # Download
            for i in range(swarm_size):
                if i != self.ID:
                    # If basestation log is not empty
                    if base.log[i].x != []:
                        # if the last time vehicle i communicated with self was before i communicated with base
                        # i.e. if the data for vehicle i on the base station is newer than on self
                        if self.time_stamps[i] < base.time_stamps[i] or elps_time == 0:
                            # Download i data
                            self.set_loc_pos(i, [base.log[i].x[-1], base.log[i].y[-1], base.log[i].z[-1]])
                            # update last time self received data on vehicle i = when vehicle i communicated with basestation
                            self.time_stamps[i] = base.time_stamps[i]
                            # Add vehicle to loc_vehicles, as only < is used in the condition, if self.time_stamp = basestation.timestamp
                            # No download occurs as there was been no update acc or sat from vehicle i since the last surface.
                            if self.loc_vehicles[i] == 0:
                                self.loc_vehicles[i] = 1
                        # if timestamps are equal there has been no update from vehicle i since last surface and hence data is uncertain
                        elif self.time_stamps[i] == base.time_stamps[i]:
                            if self.loc_vehicles[i] == 1:
                                self.loc_vehicles[i] = 0
                        # if self.time_stamps[i] > base.time_stamps the vehicle has communicated underwater and the data is newer than that on the base station


    def time_checks(self, elps_time, config):
        # If AUV is still diving after x% of max time permitted underwater, force to surface.
        if elps_time - self.t_state_change > config.t_uw * 0.5 and self.state == 0:
            # Surface
            self.set_state(1)
            self.waypoints = [self.waypoints[self.current_waypoint][0:2]+[0]]
            self.current_waypoint = 0

        if elps_time - self.t_state_change > config.t_uw * 0.9:
            # Surface NOW
            self.set_state(2)
            self.waypoints = [[self.x, self.y, 0]]
            self.current_waypoint = 0


    def set_waypoint(self, target, elps_time, config):
        if self.state != 2:
            # If the distance to a waypoint > the set maximum distance for a single dive, a dive of dive_dist (xy) is perfromed in
            # the same direction of the original waypoint. This means the AUV has a chance of reaching depth instead of diving at
            # a shallow angle to a waypoint multiple km away.
            dist_to_target = dist([self.x, self.y], target, 2)
            if dist_to_target > config.dive_dist:
                dive_x = (target[0] - self.x) * (config.dive_dist / dist_to_target)
                dive_y = (target[1] - self.y) * (config.dive_dist / dist_to_target)
                if self.state == 0:
                    self.waypoints = [[self.x + (dive_x), self.y + (dive_y), config.dive_depth]]
                elif self.state == 1:
                    self.waypoints = [[self.x + (dive_x), self.y + (dive_y), 0]]
            else:
                if self.state == 0:
                    self.waypoints = [target + [config.dive_depth]]
                elif self.state == 1:
                    self.waypoints = [target + [0]]


    def move_to_waypoint(self):
        curr_wayp = self.waypoints[self.current_waypoint]
        dist_xyz = np.array([abs(self.x - curr_wayp[0]), abs(self.y - curr_wayp[1]), abs(self.z - curr_wayp[2])])
        dist_mag = sqrt(sum(i**2 for i in dist_xyz))
        # If the AUV is within xm of the waypoint
        if dist_mag < self.config.accept_rad and self.state != 2:
            # If it isn't the last waypoint
            if curr_wayp != self.waypoints[-1]:
                # Load the next waypoint
                self.current_waypoint = self.current_waypoint + 1
                curr_wayp = self.waypoints[self.current_waypoint]
                # update distance variables for new waypoint
                dist_xyz = np.array([abs(self.x - curr_wayp[0]), abs(self.y - curr_wayp[1]), abs(self.z - curr_wayp[2])])
            else:
                # Set waypoint for vehicle to surface
                self.set_state(1)
                self.waypoints = [[curr_wayp[0], curr_wayp[1], 0]]
                self.current_waypoint = 0
                curr_wayp = self.waypoints[self.current_waypoint]
                dist_xyz = np.array([abs(self.x - curr_wayp[0]), abs(self.y - curr_wayp[1]), abs(self.z - curr_wayp[2])])

        # if self needs to move in the xy plane
        if self.x != curr_wayp[0] or self.y != curr_wayp[1]:
            # Set yaw demand to the direction of the vector from current position to waypoint
            self.set_yaw_demand(find_dir(self.x, self.y , curr_wayp[0], curr_wayp[1]))

        # If depth requirement is not met
        if self.z != curr_wayp[2]:
            # Work out distance in xy plane to work out sufficient dive angle
            dist_xy = sqrt(dist_xyz[0]**2 + dist_xyz[1]**2)
            # when dist_xy = 0, errors
            if dist_xy == 0 and self.z > curr_wayp[2]:
                self.set_pitch_demand(self.config.max_pitch)
            elif dist_xy == 0 and self.z < curr_wayp[2]:
                self.set_pitch_demand(-self.config.max_pitch)
            else:
                self.set_pitch_demand(degrees(atan(((self.z - curr_wayp[2]) / dist_xy))))

    # Models the reaction of the AUV to changes in demand with simple exponentials
    def plant(self, time_step):
        #################################### CHANGES TO YAW ###############################################

        #Quicker to go all the way round or pass through 0/360
        if self.yaw_demand - self.yaw > 180 or self.yaw_demand - self.yaw < 0 and self.yaw_demand - self.yaw > -180:
            yaw_rate = -4*(abs(self.yaw - self.yaw_demand))*exp(-1)      # Rotate Clockwise
        else:
            yaw_rate = 4*(abs(self.yaw - self.yaw_demand))*exp(-1)       # Rotate anti-clockwise

        if abs(yaw_rate) > self.config.max_yaw_rate:
            yaw_rate = (yaw_rate / abs(yaw_rate)) * self.config.max_yaw_rate

        self.set_yaw(self.yaw + (yaw_rate * time_step))

        #################################### CHANGES TO PITCH ###############################################

        pitch_rate = (self.pitch_demand - self.pitch)*exp(-1)
        if abs(pitch_rate) > self.config.max_pitch_rate:
            pitch_rate = (pitch_rate / abs(pitch_rate)) * self.config.max_pitch_rate

        self.set_pitch(self.pitch + (pitch_rate * time_step))

        #################################### CHANGES TO VEL ###############################################
        if self.v < self.v_demand:
            acc = abs(self.v - self.v_demand)*exp(-0.01)
        elif self.v > self.v_demand:
            acc = -abs(self.v - self.v_demand) * exp(-0.01)
        else:
            acc = 0

        self.set_v(self.v + (acc * time_step))

    def dead_reckoner(self, time_step):
        self.prev_x = self.x
        self.prev_y = self.y

        self.x = self.x + (self.vx * time_step)
        self.y = self.y + (self.vy * time_step)
        self.z = self.z - (self.vz * time_step)

        if self.z > 0:
            self.z = 0

        self.set_loc_pos(self.ID, [self.x, self.y, self.z])

    def actual_location(self, time_step):
        # Simulation of dead reckoning drift error
        pass

    def payload(self, t, time_step=0.5):
        global_max_loc = [500 + (0.078 * t * time_step), 500]
        dist = np.sqrt(((global_max_loc[0] - self.x) ** 2) + ((global_max_loc[1] - self.y) ** 2))
        if dist > 850:
            self.measurement = 0
        else:
            self.measurement = 25000 * (1 / (125 * np.sqrt(2 * pi))) * np.exp((-((dist) ** 2)) / (2 * (125 ** 2)))

    def go(self, config, elps_time = 0):
        # Have to skip move_to_waypoint in velocity validation.
        if config.sim_sub_type != 3:
            self.move_to_waypoint()
        self.plant(config.time_step)
        self.dead_reckoner(config.time_step)
        self.payload(elps_time, config.time_step)
        self.logger()

    ############################################### SETTERS ############################################################

    def set_yaw(self, yaw):
        if yaw < 0:
            self.yaw = yaw + 360
        elif yaw > 360:
            self.yaw = yaw - 360
        else:
            self.yaw = yaw

        # Recalculates vx,vy,vz
        self.set_v()

    def set_yaw_demand(self, yd):
        self.yaw_demand = yd

    def set_pitch(self, pitch):
        if abs(pitch) > self.config.max_pitch:
            self.pitch = (pitch / abs(pitch)) * self.config.max_pitch
        else:
            self.pitch = pitch

        # Recalculates vx,vy,vz
        self.set_v()

    def set_pitch_demand(self, pd):
        if abs(pd) > self.config.max_pitch:
            self.pitch_demand = (self.pitch_demand / abs(self.pitch_demand)) * self.config.max_pitch
        else:
            self.pitch_demand = pd

    def set_v(self, v = -999):
        if v == -999:
            v = self.v
        if v > self.config.max_v:
            self.v = self.config.max_v
        elif v < self.config.min_v:
            self.v = self.config.min_v
        else:
            self.v = v

        v_xy = self.v * cos(radians(self.pitch))
        self.vx = v_xy * cos(radians(self.yaw))
        self.vy = v_xy * sin(radians(self.yaw))
        self.vz = self.v * sin(radians(self.pitch))

    def set_v_demand(self, vd):
        if vd >= 0:
            self.v_demand = vd

    def set_loc_pos(self, ID, pos):
        self.loc_pos[ID] = pos
        self.loc_dist[ID] = np.sqrt(sum((pos - np.array([self.x, self.y, self.z])) ** 2))

    def set_state(self, s):
        if self.state == 1 and s == 0 and self.z < -0.5:
            print('INVALID STATE CHANGE: Attempted to move from surfacing to diving at z %f m' % self.z)
            raise
        elif self.state == 2 and s == 0 and self.z < -0.5:
            print('INVALID STATE CHANGE: Attempted to move from immediate surfacing to diving at z %f m' % self.z)
            raise
        elif self.state == 2 and s == 1:
            print('INVALID STATE CHANGE: Attempted to change from immediate surface to surface.')

        self.state = s

    ####################################################################################################################

    def logger(self):
        self.log.x.append(self.x)
        self.log.y.append(self.y)
        self.log.z.append(self.z)
        self.log.v.append(self.v)
        self.log.yaw.append(self.yaw)
        self.log.pitch.append(self.pitch)
        self.log.v_demand.append(self.v_demand)
        self.log.yaw_demand.append(self.yaw_demand)
        self.log.pitch_demand.append(self.pitch_demand)
        self.log.loc_pos.append(np.copy(self.loc_pos))
        self.log.measurement.append(self.measurement)
        self.log.x_demand.append(self.waypoints[self.current_waypoint][0])
        self.log.y_demand.append(self.waypoints[self.current_waypoint][1])
        self.log.z_demand.append(self.waypoints[self.current_waypoint][2])

    def __del__(self):
        pass

    def __init__(self,i, Swarm_Size, start_x, start_y, start_z, start_yaw):
        # Loading configurable parameters
        self.config = sim_config('config/vehicle_config.csv')
        self.ID = i
        # Initialising
        self.x = start_x
        self.y = start_y
        self.z = start_z
        self.yaw = start_yaw        # Heading in degrees 0 is along +ve x axis
        self.pitch = 0              # +ve = Nose up (rise), -ve = Nose down (Dive)
        self.set_v(0.5)                # Velocity in m/s
        self.set_pitch_demand(self.pitch)
        self.yaw_demand = self.yaw
        self.v_demand = self.v
        self.sat_commd = 0
        self.log = Log()
        self.state = 0
        self.t_state_change = 0
        self.payload(0)
        self.current_waypoint = 0
        self.waypoints = [[start_x,start_y,0]]
        # Swarm properties
        # if Swarm_Size > 1:
        self.time_stamps = list([-1 for i in range(0,Swarm_Size)])
        self.loc_vehicles = np.zeros(Swarm_Size)
        self.loc_v = np.zeros(Swarm_Size)
        self.loc_dist = np.zeros(Swarm_Size)
        self.loc_pos = np.array([np.zeros(Swarm_Size), np.zeros(Swarm_Size), np.zeros(Swarm_Size)]).transpose()
        self.loc_yaw = np.zeros(Swarm_Size)
        self.loc_pitch = np.zeros(Swarm_Size)
        self.loc_measurements = np.zeros(Swarm_Size)
        self.set_loc_pos(self.ID, [self.x, self.y, self.z])

        self.logger()



# Notes

# [1] I'm pretty sure acoustic data will always be newer than Sat data hence this check is redundant
#      but with vehicles coming in and out of range and with the acoustic delay of 100000s of robots I can't say I'm 100% sure.