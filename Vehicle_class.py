import sys
sys.path.insert(0, "../gen")
from math import sqrt, atan, exp, cos, sin, radians, degrees, ceil, floor, pi, isnan
from numpy import abs
import numpy as np
from log import Log
from Config_class import *
from Acc_channel_class import *
from latlon_util import find_dir, find_dist2

class Vehicle:
    def send_acc_msg(self, Acc_comms, elps_time, config, Swarm):
        """
        Sends message to the acoustic channel then triggers transmit message which determines which vehicles receive the messages
        based on range from source. No current message collision.
        :param Acc_comms: Acoustic channel
        :param elps_time: Time stamp for message
        :param config: Simulation config, used for transmit message
        :param Swarm: List of vehicle_class objects
        """
        if self.state != 3:
            Acc_comms.msg = Msg(self.ID, elps_time, self.v, [self.lon, self.lat, self.z] , self.yaw, self.pitch)
            Acc_comms.transmit_msg(Swarm, config)

    def receive_acc_msg(self, Acc_comms):
        """
        Checks the acoustic channel for a message.
        Updates if the message is newer than data held by the vehicle
        :param Acc_comms: Acoustic Channel
        """
        if Acc_comms.receive_msg[self.ID] == 1:
            msg = Acc_comms.msg
            self.loc_vehicles[msg.ID] = 1
            if msg.sent_time >= self.time_stamps[msg.ID]: #[1]
                self.time_stamps[msg.ID] = msg.sent_time
                self.loc_v[msg.ID] = msg.v
                self.set_loc_pos(msg.ID, msg.pos)
                self.loc_yaw[msg.ID] = msg.yaw
                self.loc_pitch[msg.ID] = msg.pitch

    def sat_comms(self, base, config, elps_time):
        """
        Simulates satellite communication. The vehicle must first stop and pitch entirely nose down in order to receive and
        send data. There is a delay of 3 minutes to simulate waiting for a satellite fix. The variable sat_commd ensures the
        vehicle does not repeat this process while on the surface, this is reset after diving below 0.5m in normal operation
        or after t = 0.5t_uw in surface ops.
        :param base: Base station which contains the data uploaded by all members of the swarm
        :param config: Simualtion configuration
        :param elps_time: Time elapsed in simulation, used as a timestamp for data
        """
        if config.comms != 2:
            if self.sat_commd == 0:
                if self.z > -0.1 and self.sat_commd == 0 and self.state != 3:
                    self.set_v_demand(0)
                    self.set_state(3, elps_time)

                if self.v < 0.001 and self.state == 3 and self.z > -0.1:
                    self.set_pitch_demand(self.config.max_pitch)

                if abs(self.pitch - self.config.max_pitch) < 0.001 and self.v < 0.001 and self.z > -0.1 and self.sat_commd == 0 and self.state == 3:
                    self.sat_up(base, elps_time)
                    self.sat_down(config, base, config.swarm_size, elps_time)
                    self.sat_commd = 1

            if self.state == 3 and self.sat_commd == 1 and (elps_time - self.log.sat_time_stamps[-1]) >= config.t_sat / config.time_step:
                if config.sim_type == 1 and config.sim_sub_type == 0:
                    self.set_pitch_demand(0)
                    if abs(self.pitch) < 0.001:
                        self.set_state(0, elps_time)
                else:
                    self.set_state(0,elps_time)

        else:
            # In the ideal communication case:
            if self.z > -0.1 and self.sat_commd == 0:
                self.set_state(3, elps_time)
                self.sat_up(base, elps_time)
                self.sat_down(config, base, config.swarm_size, elps_time)
                self.sat_commd = 1
                self.set_state(0, elps_time)


    def sat_up(self, base, elps_time):
        if self.z > -0.1:
            # Upload
            base.log[self.ID].lon.append(self.lon)
            base.log[self.ID].lat.append(self.lat)
            base.log[self.ID].z.append(self.z)
            base.log[self.ID].v.append(self.v)
            base.time_stamps[self.ID] = elps_time
            self.log.sat_time_stamps.append(elps_time)

    def sat_down(self, config, base, swarm_size, elps_time):
        if self.z > -0.1:
            # Download
            for i in range(swarm_size):
                if i != self.ID:
                    # If basestation log is not empty
                    if base.log[i].lon != []:

                        # if the last time vehicle i communicated with self was before i communicated with base
                        # i.e. if the data for vehicle i on the base station is newer than on self
                        if self.time_stamps[i] < base.time_stamps[i] or elps_time == 0:
                            # Download i data
                            self.set_loc_pos(i, [base.log[i].lon[-1], base.log[i].lat[-1], base.log[i].z[-1]])
                            # update last time self received data on vehicle i = when vehicle i communicated with basestation
                            self.time_stamps[i] = base.time_stamps[i]
                            # Add vehicle to loc_vehicles, as only < is used in the condition, if self.time_stamp = basestation.timestamp
                            # No download occurs as there was been no update acc or sat from vehicle i since the last surface.
                            if self.loc_vehicles[i] == 0:
                                self.loc_vehicles[i] = 1
                        # if timestamps are equal there has been no update from vehicle i since last surface and hence data is uncertain
                        # config.comms = 0 = sat only case however because all the AUVs dive in order the first one surfaces, checks the
                        # base and its got all the same info as before hence the AUV thinks all the others have died.
                        elif self.time_stamps[i] == base.time_stamps[i] and config.comms != 0 and config.comms != 2:
                            if self.loc_vehicles[i] == 1:
                                self.loc_vehicles[i] = 0
                        # if self.time_stamps[i] > base.time_stamps the vehicle has communicated underwater and the data is newer than that on the base station


    def time_checks(self, elps_time, config):
        if self.get_t_uw(elps_time) > config.t_uw * 0.9 and self.state != 2:
            self.set_state(2, elps_time)
            self.sat_commd = 0
            if self.waypoints[0][2] == 0:
                self.waypoints = [[self.lon,self.lat,0],[(self.lon + self.waypoints[0][0]) / 2,(self.lat + self.waypoints[0][1]) / 2,config.dive_depth]] + self.waypoints
            else:
                self.waypoints = [[self.lon, self.lat, 0]] + self.waypoints

    def set_waypoint(self, target, config):
        if self.state != 2:
            # If the distance to a waypoint > the set maximum distance for a single dive, a dive of dive_dist (xy) is perfromed in
            # the same direction of the original waypoint. This means the AUV has a chance of reaching depth instead of diving at
            # a shallow angle to a waypoint multiple km away.
            dist_to_target =  find_dist2([self.lon, self.lat], (target[0], target[1]))
            # Saturates the distance an AUV can move in one dive
            if dist_to_target > config.dive_dist:
                dive_lon = (target[0] - self.lon) * (config.dive_dist / dist_to_target)
                dive_lat = (target[1] - self.lat) * (config.dive_dist / dist_to_target)
                self.waypoints = [[self.lon + dive_lon, self.lat + dive_lat, config.dive_depth]]
            else:
                self.waypoints = [target + [config.dive_depth]]

    def move_to_waypoint(self, elps_time, config):
        dist_mag =  find_dist2((self.lon, self.lat), (self.waypoints[0][0], self.waypoints[0][1]))
        # If the AUV is within xm of the waypoint and 1m within sepcified depth
        if dist_mag < self.config.accept_rad and abs(self.z - self.waypoints[0][2]) < 1:
            self.next_waypoint(config, elps_time)
        else:
            self.set_v_demand(self.config.max_v / 2.0)


        # if self needs to move in the xy plane
        #if self.lon != self.waypoints[0][0] or self.lat != self.waypoints[0][1]:
        # Set yaw demand to the direction of the vector from current position to waypoint
        self.set_yaw_demand(find_dir((self.lon, self.lat), (self.waypoints[0][0], self.waypoints[0][1])))

        # If depth requirement is not met
        if self.z != self.waypoints[0][2]:
            # Work out distance in xy plane to work out sufficient dive angle
            dist_xy =  find_dist2((self.lon, self.lat), (self.waypoints[0][0], self.waypoints[0][1]))
            # when dist_xy = 0, errors
            if dist_xy == 0 and self.z > self.waypoints[0][2]:
                self.set_pitch_demand(self.config.max_pitch)
            elif dist_xy == 0 and self.z < self.waypoints[0][2]:
                self.set_pitch_demand(-self.config.max_pitch)
            else:
                self.set_pitch_demand(degrees(atan(((self.z - self.waypoints[0][2]) / dist_xy))))

    def next_waypoint(self, config, elps_time):
        if self.state == 2 or self.waypoints[0][2] == 0:
            self.set_state(0, elps_time)
        if len(self.waypoints) == 1:
                # Loiter on surface
                self.set_state(2, elps_time)
                self.waypoints = self.waypoints + [[self.waypoints[0][0], self.waypoints[0][1], 0]]
                self.set_v_demand(0)
        self.sat_commd = 0
        self.waypoints.pop(0)



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

        self.set_yaw(self.yaw + ((self.v / self.config.max_v) * yaw_rate * time_step))

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

    def dead_reckoner(self, config):
        self.inc_lat(config)
        self.inc_lon(config)
        self.z = self.z - (self.vz * config.time_step)

        if self.z > 0:
            self.z = 0

        self.set_loc_pos(self.ID, [self.lon, self.lat, self.z])

    def inc_lat(self, config):
        dS = self.vy * config.time_step # meters
        dlat = dS / 111111 # Approx length of 1 degree of latitude in km
        self.lat = self.lat + dlat

    def inc_lon(self, config):
        m_per_deglon = np.cos(np.radians(self.lat)) * 111111
        dlon = ( self.vx * config.time_step ) / m_per_deglon
        self.lon = self.lon + dlon

    def actual_location(self, time_step):
        # Simulation of dead reckoning drift error
        pass

    def payload(self, config, t):
        if config.feature_move == 1:
            global_max_loc = [500 + (0.078 * t * config.time_step), 500]
        else:
            global_max_loc = [500,500]
        dist2max = find_dist2((self.lon, self.lat), (global_max_loc[0], global_max_loc[1]))
        if dist2max > 850:
            self.measurement = 0
        else:
            self.measurement = 25000 * (1 / (125 * np.sqrt(2 * pi))) * np.exp((-((dist2max) ** 2)) / (2 * (125 ** 2)))

    def go(self, config, elps_time = 0):
        # Have to skip move_to_waypoint in velocity validation.
        if self.state != 3:
            if config.sim_sub_type != 3:
                self.move_to_waypoint(elps_time, config)
        self.plant(config.time_step)
        self.dead_reckoner(config)
        if config.feature_monitoring == 1:
            self.payload(config, elps_time)

    ############################################### SETTERS / GETTERS ############################################################

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
            self.pitch_demand = (pd / abs(pd)) * self.config.max_pitch
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
        self.vx = v_xy * sin(radians(self.yaw))
        self.vy = v_xy * cos(radians(self.yaw))
        self.vz = self.v * sin(radians(self.pitch))

    def set_v_demand(self, vd):
        if vd >= 0:
            self.v_demand = vd

    def set_loc_pos(self, ID, pos):
        self.loc_pos[ID] = pos
        self.loc_dist[ID] = find_dist2((self.lon, self.lat), (pos[0], pos[1]))

    def set_state(self, s, elps_time):
        # if self.state == 1 and s == 0:
        #     print('INVALID STATE CHANGE: Attempted to move from surfacing to diving')
        #     raise
        # elif self.state == 2 and s == 0:
        #     print('INVALID STATE CHANGE: Attempted to move from immediate surfacing to diving')
        #     raise
        # elif self.state == 2 and s == 1:
        #     print('INVALID STATE CHANGE: Attempted to change from immediate surface to surface.')
        #     raise
        self.t_state_change = elps_time
        self.state = s

    def get_t_uw(self, elps_time):
        if self.state == 0:
            t_uw = elps_time - self.t_state_change # JON SNOW KNOWS NOTHING / He's dead and alive again
        else:
            t_uw = 0
        return t_uw

    ####################################################################################################################

    def logger(self, elps_time):
        # self.log.x.append(self.x)
        # self.log.y.append(self.y)
        self.log.z.append(self.z)
        self.log.v.append(self.v)
        self.log.yaw.append(self.yaw)
        self.log.pitch.append(self.pitch)
        self.log.v_demand.append(self.v_demand)
        self.log.yaw_demand.append(self.yaw_demand)
        self.log.pitch_demand.append(self.pitch_demand)
        self.log.loc_pos.append(np.copy(self.loc_pos))
        self.log.measurement.append(self.measurement)
        self.log.lon_demand.append(self.waypoints[0][0])
        self.log.lat_demand.append(self.waypoints[0][1])
        self.log.z_demand.append(self.waypoints[0][2])
        self.log.state.append(self.state)
        self.log.time_uw.append(self.get_t_uw(elps_time))
        self.log.lat.append(self.lat)
        self.log.lon.append(self.lon)

    def __del__(self):
        pass

    def __init__(self, config,i, Swarm_Size, start_lon, start_lat, start_z, start_yaw):
        # Loading configurable parameters
        self.config = sim_config('config/vehicle_config.csv')
        self.ID = i
        # Initialising
        self.lat = start_lat
        self.lon = start_lon
        self.z = start_z
        # self.x = start_x
        # self.y = start_y
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
        self.t_uw = 0
        self.payload(config, 0)
        self.waypoints = [[start_lon, start_lat, 0]]
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
        self.set_loc_pos(self.ID, [self.lon, self.lat, self.z])


        self.logger(0)



# Notes

# [1] I'm pretty sure acoustic data will always be newer than Sat data hence this check is redundant
#      but with vehicles coming in and out of range and with the acoustic delay of 100000s of robots I can't say I'm 100% sure.