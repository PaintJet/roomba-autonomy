#!/usr/bin/env python

from __future__ import print_function

# ROS imports
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Twist
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64
# Message for map
from nav_msgs.msg import OccupancyGrid

# ROS tf2 imports
import tf
import tf_conversions # Use to convert from Euler angles to quaternions


from pfilter import ParticleFilter, independent_sample, squared_error, gaussian_noise
import numpy as np
from scipy.stats import uniform, norm

import time

from map_utils import Map






class UWBParticleFilter:
    # Implementation of a particle filter to predict the pose of the robot given distance measurements from the modules
    # anchor_positions is the ground truth location of all of the anchors
    # tag_transforms is a 2d numpy array storing the [x,y] coordinates of each tag in the robot frame
    def __init__(self, num_sensors, anchor_positions, tag_transforms, laser_transform, initial_position, occ_grid, update_rate):
        # Number of particles
        self.n = 100
        # Sensor covariance for range measurement of UWB modules
        self.UWB_COVARIANCE = 0.5
        self.IMU_COVARIANCE = 0.05

        self.update_rate = float(update_rate) #Hz

        # Members of the state
        self.state_desc = ["x", "y", "theta"]

        # Number of sensors that we are reading data from
        self.num_sensors = num_sensors

        #TODO Dynamically update this value
        self.num_scans = 90

        self.map = occ_grid


        # Prior sampling function for each of the state variables
        # We assume uniform distribution over [0 m, 20 m]
        self.prior_fn = independent_sample([
            norm(loc = initial_position[0], scale = 2).rvs,
            norm(loc = initial_position[0], scale = 2).rvs,
            uniform(loc =   0, scale = 2*np.pi).rvs
        ])

        self.num_anchors = anchor_positions.shape[0]
        self.anchor_positions = anchor_positions

        self.pf = ParticleFilter(
            n_particles = self.n,
            prior_fn = self.prior_fn,
            observe_fn = self.observation_function,
            weight_fn = self.weight_function,
            dynamics_fn = self.dynamics_function,
            noise_fn = lambda x: gaussian_noise(x, sigmas=[0.1, 0.1, 0.1])
        )


        self.num_tags = tag_transforms.shape[0]
        self.tag_transforms = tag_transforms
        self.laser_transform = laser_transform

        self.pf.update()


    # weight_function
    # Accepts the hypothetical observations for each particle and real observations for all of the sensors and
    # computes a weight for the particle
    # hyp_observed - (nxh) matrix containing the hypothetical observation for each particle
    # real_observed - (h,) vector containing the real observation
    # returns a (n,) vector containing the weights for each of the particles
    def weight_function(self, hyp_observed, real_observed):
        # Create output vector of dimension (n,)
        particle_weights = np.zeros(shape = (hyp_observed.shape[0],), dtype = np.float32)

        print(real_observed)

        # First split UWB and IMU data into separate arrays
        hyp_observed_uwb = hyp_observed[:,:-1]
        hyp_observed_imu = hyp_observed[:,-1:]

        real_observed_uwb = real_observed[0,:-1]
        real_observed_imu = real_observed[0,-1:]

        print(real_observed_imu)

        # Assume the range measurements for the UWB modules has gaussian noise
        # Iterate over each particle

        #TODO Real observation does not have particle dimension - need to map all parameters with one fixed input
        # def calc_particle_weight(hyp_observation_uwb, hyp_observation_imu):
        #     # Calculate the gaussian pdf for the difference between the real and expected sensor measurements for all unmasked elements in the array
        #     # Since all of the sensor measurements are independent, we can simply multiply the 1-D probabilities
        #     # We construct a gaussian with a mean of 0 and a variance empirically determined for the UWB modules
        #     uwb_measurement_probabilities = norm(0, self.UWB_COVARIANCE).pdf(hyp_observation_uwb[~real_observed_uwb.mask[0]] - real_observed_uwb.data[0][~real_observed_uwb.mask[0]])
        #     uwb_particle_weight = np.prod(uwb_measurement_probabilities, axis = 0)
        #     #print("UWB weighting  time: {}".format(time.clock() - start))

        #     imu_measurement_probability = norm(0, self.IMU_COVARIANCE).pdf(hyp_observation_imu[~real_observed_imu.mask[0]] - real_observed_imu.data[0][~real_observed_imu.mask[0]])
        #     imu_particle_weight = np.prod(imu_measurement_probability, axis = 0)

        #     return uwb_particle_weight + imu_particle_weight

        # particle_weights = map(calc_particle_weight, hyp_observed_uwb, hyp_observed_imu)
        for i in range(hyp_observed.shape[0]):

            #print("UWB weighting  time: {}".format(time.clock() - start))

            #imu_measurement_probability = norm(0, self.IMU_COVARIANCE).pdf(hyp_observed_imu[i][~real_observed_imu.mask[0]] - real_observed_imu.data[0][~real_observed_imu.mask[0]])
            #imu_particle_weight = np.prod(imu_measurement_probability, axis = 0)


            lidar_particle_weight = self.calc_lidar_particle_weight_likelihood_field(hyp_observed[i], real_observed[0])


            # print("{} {} {}".format(hyp_observed_imu[i], real_observed_imu, imu_measurement_probability))

            #particle_weights[i] = calc_uwb_particle_weight(hyp_observed[i], real_observed[0]) + imu_particle_weight + lidar_particle_weight
            particle_weights[i] = lidar_particle_weight

        # for one, two in zip(particle_weights, particleWeights):
        #     if(one != two):
        #         delta = one - two
        #         print("BROKEN {}".format(delta))

        return particle_weights

    def calc_uwb_particle_weight(self, hyp_observed, real_observed):
        # Calculate the gaussian pdf for the difference between the real and expected sensor measurements for all unmasked elements in the array
        # Since all of the sensor measurements are independent, we can simply multiply the 1-D probabilities
        # We construct a gaussian with a mean of 0 and a variance empirically determined for the UWB modules
        uwb_measurement_probabilities = norm(0, self.UWB_COVARIANCE).pdf(hyp_observed[~real_observed.mask] - real_observed.data[~real_observed.mask])
        return np.prod(uwb_measurement_probabilities, axis = 0)

    def calc_imu_particle_weight(self, hyp_observed, real_observed):
        imu_measurement_probability = norm(0, self.IMU_COVARIANCE).pdf(hyp_observed[~real_observed.mask] - real_observed.data[~real_observed.mask])
        return  np.prod(imu_measurement_probability, axis = 0)

    def calc_lidar_particle_weight_likelihood_field(self, hyp_observed, real_observed):
        # The hyp_observed vector stores the estimated 2d pose of the lidar in the world frame
        #TODO this is a hacky way to get the pose that needs to be fixed later
        # Pose of the lidar retrieved from hype observed
        lidar_pose = hyp_observed[self.num_tags*self.num_anchors : self.num_tags*self.num_anchors + 3]

        #TODO Parametrize these constants
        z_hit = 1
        z_rand = 0
        self.sigma_hit = 0.005
        self.lidar_range_max = 10

        # Particle weight
        p = 1.0

        # Calculate constant terms for the particle weight
        z_hit_denom = 2 * self.sigma_hit * self.sigma_hit
        z_rand_mult = 1.0/self.lidar_range_max



        for i in range(self.num_scans):
            if(real_observed.mask[self.num_tags*self.num_anchors + i] == 0):
                # Retrieve range & bearing in easy to access variables
                obs_range = real_observed[self.num_tags*self.num_anchors + i]
                obs_bearing = real_observed[self.num_tags*self.num_anchors + self.num_scans + i]

                pz = 0.0;

                # Compute the endpoint of the beam
                beam_x = lidar_pose[0] + obs_range * np.cos(lidar_pose[2] + obs_bearing);
                beam_y = lidar_pose[1] + obs_range * np.sin(lidar_pose[2] + obs_bearing);

                # Get occupancy grid coordinates
                #TODO Move this to map utils
                mi = int(np.floor((beam_x - 0) / self.map.resolution + 0.5))
                mj = int(np.floor((beam_y - 0) / self.map.resolution + 0.5))

                # Part 1: Get distance from the hit to closest obstacle.
                # Off-map penalized as max distance
                if(not self.map.on_map( mi, mj)):
                    z = self.map.MAX_DISTANCE
                else:
                    z = self.map.distances[mi, mj] # Retrieve distance to nearest neighbor

                # Gaussian model
                # NOTE: this should have a normalization of 1/(sqrt(2pi)*sigma)
                pz += z_hit * np.exp(-(z * z) / z_hit_denom)

                # Part 2: random measurements
                pz += z_rand * z_rand_mult

                # TODO: outlier rejection for short readings

                assert pz <= 1.0
                assert pz >= 0.0

                #      p *= pz;
                # here we have an ad-hoc weighting scheme for combining beam probs
                # works well, though...
                p += pz*pz*pz

        return p







    # observation_function
    # Accepts the state of all the particles and returns the predicted observation for each one
    # particles states - n x d array: n is the number of particles and d is state dimension
    def observation_function(self, particle_states):
        # Create output array of dimension n x h
        # n is the numer of particles
        # h is the dimension of the observation
        expected_observations = np.zeros(shape = (self.n, self.num_sensors))

        # Calculated expected observations
        num_uwb_measurements = self.num_tags*self.num_anchors
        for i in range(self.n):
            # Update UWB predicted observations
            expected_observations[i][0:num_uwb_measurements] = self.uwb_observation_function(particle_states[i])

            # Don't update the lidar observations because we are using the likelihood model to generate the weights for the particles
            # If we switch to the beam model, we will have to update expected observations using raycasts
            expected_observations[i][num_uwb_measurements:num_uwb_measurements+3] = self.lidar_observation_function(particle_states[i])

            # Add the IMU observation, which is just the orientation stored in the pf state
            expected_observations[i][-1] = self.imu_observation_function(particle_states[i])

        #print("Loop time: {}".format(time.clock()-start))

        return expected_observations

    # uwb_observation_function
    # Accepts the state of a given particle and returns the UWB observations for that particle
    def uwb_observation_function(self, particle_state):
        expected_observations = np.zeros(shape = (self.num_tags*self.num_anchors))
        for j in range(self.num_tags):
            for k in range(self.num_anchors):
                #print("{} {}".format(particle_states[i], self.anchor_positions[j]), end = "\n")

                # Calculate expected position of the tag in the world frame
                x_tag = self.tag_transforms[j][0] * np.cos(particle_state[2]) - self.tag_transforms[j][1] * np.sin(particle_state[2]) + particle_state[0]
                y_tag = self.tag_transforms[j][0] * np.sin(particle_state[2]) + self.tag_transforms[j][1] * np.cos(particle_state[2]) + particle_state[1]
                expected_tag_positon = np.array([x_tag, y_tag])

                # Expected observation is the distance from the tag to the anchor
                expected_observations[self.num_anchors * j + k] = np.linalg.norm(expected_tag_positon - self.anchor_positions[k])
        return expected_observations


    def imu_observation_function(self, particle_state):
        return particle_state[2]

    # Currently, the lidar observation function simply returns the pose of the lidar given the estimated particle pose
    def lidar_observation_function(self, particle_state):
        # Get 2D pose of LIDAR relative to map for the current particle
        x_lidar = self.laser_transform[0] * np.cos(particle_state[2]) - self.laser_transform[1] * np.sin(particle_state[2]) + particle_state[0]
        y_lidar = self.laser_transform[0] * np.sin(particle_state[2]) + self.laser_transform[1] * np.cos(particle_state[2]) + particle_state[1]
        theta_lidar = particle_state[2]
        return np.array([x_lidar, y_lidar, particle_state[2]])


    # dynamics_function
    # Accepts the state of all of the particles and returns the predicted state based on the control input to the robot
    # Applies a dynamics model to evolve the particles
    # Control input read from ROS topic cmd_vel
    # Also applies noise proportional to the control input
    # Arguments:
    #   particles_states - (n,d) vector containing the state of all of the particles
    #  returns predicted particle_states (n,d) vector with evolved particles
    def dynamics_function(self, particle_states):
        # Get current cmd_vel
        cmd_vel = rospy.wait_for_message("/cmd_vel", Twist)

        # Time step between updates
        delta_t = 1/self.update_rate

        # Calculate change in angle and arc length traversed
        #TODO: FIGURE OUT WHY THESE CONSTANTS ARE REQUIRED
        delta_theta = cmd_vel.angular.z * delta_t
        delta_s = cmd_vel.linear.x * delta_t

        #print("delta_x: {} \t delta_y: {} \t delta_theta: {}".format(delta_x, delta_y, delta_theta))


        # Init variables to store translation in x and y directions in the robot frame
        # delta_y is the forward direction of the robot
        # delta_x is the right direction of the robot
        # NOTE: This coordinate frame does not coincide with the ROS coordinate frame of the robot
        # ROS coordinate frame is rotated 90 degrees counter-clockwise
        delta_x_robot = 0.0
        delta_y_robot = 0.0

        # This is a singularity where the robot only moves forward
        # Radius of circle is infinite
        if delta_theta == 0:
            delta_y_robot = delta_s
        else:
            # Radius of the circle along which the robot is travelling
            r = delta_s/delta_theta

            # Next, calculate the translation in the robot frame
            delta_x_robot = r*(np.cos(delta_theta) - 1)
            delta_y_robot = r*np.sin(delta_theta)

        for i in range(particle_states.shape[0]):

            # Transform x and y translation in robot frame to world coordinate frame for each particle
            # TODO REMOVE HACKY pi/2 fix
            delta_x = delta_x_robot*np.cos(particle_states[i][2] - np.pi/2) - delta_y_robot*np.sin(particle_states[i][2]- np.pi/2)
            delta_y = delta_x_robot*np.sin(particle_states[i][2] - np.pi/2) + delta_y_robot*np.cos(particle_states[i][2] - np.pi/2)

            particle_states[i][0] = particle_states[i][0] + delta_x
            particle_states[i][1] = particle_states[i][1] + delta_y
            particle_states[i][2] = particle_states[i][2] + delta_theta

            # if(i == 0):
            #     print("delta_x: {} \t delta_y: {} \t delta_theta: {}".format(delta_x_robot, delta_y_robot, delta_theta))


        # Return updated particles
        return particle_states

    # update
    # Takes in the observation from the sensors and calls an update step on the particle filter
    # Accepts the observation as (4,) array containing range measurements from the anchors
    # Supports masked numpy array for partial observations
    def update(self, observation):
        self.pf.update(observation)

    def get_particle_states(self):
        return self.pf.original_particles

    def get_state(self):
        return self.pf.mean_state

# create_uwb_sensor_update_function
# Factory function that generates an observation function for each of the UWB anchors
# The observation function will craft the observation input into the particle filter and write it to the global sensor_measurements vector
# sensor_vector_pos - Start index for the UWB measurements in the sensor measurement vector
# tag_number - Which tag on the robot is it
# anchor_number - Which anchor in the environment is it
# num_tags - Total number of UWB tags on the robot
# num_anchors - Total number of anchors in the environment
def create_uwb_sensor_update_function(sensor_vector_pos, tag_number, anchor_number, num_sensors, num_tags, num_anchors, pf):
    def update_sensor_vector(message):
        # Get global arrays for sensor measurements
        global sensor_measurements, sensor_measurements_mask

        #print("Running update for tag {} anchor {}".format(tag_number, anchor_number))
        # # Create array for particle filter observation
        # pf_input = np.zeros(shape = (num_sensors), dtype = np.float64)

        # Create array to mask observation matrix
        # mask = np.ones(shape = (num_sensors))

        # Write the distance observation to the correct index associated with the tag and anchor
        sensor_measurements[num_anchors*tag_number + anchor_number + sensor_vector_pos] = message.data

        # Mask all elements except the one for the measurement we collected
        sensor_measurements_mask[num_anchors*tag_number + anchor_number + sensor_vector_pos] = 0

        # pf.update(np.ma.masked_array(pf_input, mask = mask))

    return update_sensor_vector

# create_imu_observation_function
# Factory function that generates an observation function for the IMU
# sensor_vector pos - Array index for storing the IMU measurement in the sensor measurement vector
# The observation function will craft the observation input into the particle filter and write it to the global sensor_measurements vector
def create_imu_sensor_update_function(sensor_vector_pos):
    # Observation function that is called when data is received from the IMU
    def update_sensor_vector(message):
        # Get global arrays for sensor measurements
        global sensor_measurements, sensor_measurements_mask

        # Write the distance observation to the correct index associated with the tag and anchor
        _, _, sensor_measurements[-1] = tf_conversions.transformations.euler_from_quaternion([message.orientation.x, message.orientation.y, message.orientation.z, message.orientation.w])

        # Mask all elements except the one for the measurement we collected
        sensor_measurements_mask[-1] = 0

        # pf.update(np.ma.masked_array(pf_input, mask = mask))

    return update_sensor_vector


# create_lidar_observation_function
# Factory function that generates an observation function for the lidar laser scans
# sensor_vector pos - Start array index for the LIDAR scan data in the sensor measurement vector
# The observation function will craft the observation input into the particle filter and write it to the global sensor_measurements vector
def create_lidar_sensor_update_function(sensor_vector_pos):
    # Observation function that is called when data is received from the IMU
    def update_sensor_vector(message):
        # Get global arrays for sensor measurements
        global sensor_measurements, sensor_measurements_mask

        # Retrieve number of sensor measurements
        num_measurements = len(message.ranges)

        # Loop over all range measurements
        for i, range_measurement in enumerate(message.ranges):
            #TODO Constants
            range_max = 10
            range_min = 0

            # Calculate bearing of sensor measurement
            angle = message.angle_min + i*message.angle_increment

            # Ignore sensor measurement if it is outside of scanner range
            if(range_measurement < range_max and range_measurement > range_min):
                # Add angle and range measurement to sensor vector
                sensor_measurements[sensor_vector_pos + i] = range_measurement
                sensor_measurements[sensor_vector_pos + num_measurements + i] = angle

                # Remove mask from these elements
                sensor_measurements_mask[sensor_vector_pos + i] = 0
                sensor_measurements_mask[sensor_vector_pos + num_measurements + i] = 0

    return update_sensor_vector





if __name__ == "__main__":
    ## Variables for data logging
    # Flag to decide if data should be logged during run
    LOG_DATA = False
    log_path = "uwb-data.csv"

    # Rate to update particle filter
    # TODO determine why this is stuck at 20 Hz
    UPDATE_RATE = 2

    # Open file to store logs
    if LOG_DATA:
        f = open(log_path, "w")

    # Init ros node
    rospy.init_node("uwb_particle_filter")

    rospy.loginfo("Initializing Particle Filter Node...")

    rospy.loginfo("Retrieving map of environment...")

    # Retrieve occupancy grid
    occupancy_grid_msg = rospy.wait_for_message('/map', OccupancyGrid)

    # Create a map object
    occupancy_grid = Map(occupancy_grid_msg)


    rospy.loginfo("Successfully loaded map!")


    # NUM UWB tags
    num_tags = 3
    # NUM UWB anchors
    num_anchors = 9

    # TODO Dynamically adjust this value
    num_lidar_range_scans = 90
    lidar_range_min = 1
    lidar_range_max = 2

    # Total number of sensors - used to construct the sensor measurement array
    # Each tag receives a range measurement from each anchor, creating num_tags*num_anchors measurements
    # 1 sensor measurement from IMU - angular position
    # Number of sensor measurements for the LIDAR is 2*(number of scans) for range and bearing
    num_sensors = 1 + num_tags*num_anchors + 2*num_lidar_range_scans

    # Read the locations of the anchors from the ros messages
    # anchor0Loc = rospy.wait_for_message('/uwb/0/anchors/9205/position', Point)
    # anchor1Loc = rospy.wait_for_message('/uwb/0/anchors/9AAB/position', Point)
    # anchor2Loc = rospy.wait_for_message('/uwb/0/anchors/C518/position', Point)
    # anchor3Loc = rospy.wait_for_message('/uwb/0/anchors/D81B/position', Point)

    # Instantiate particle filter
    # pf = UWBParticleFilter(np.array([[anchor0Loc.x, anchor0Loc.y]
    #                                    #[anchor1Loc.x, anchor1Loc.y],
    #                                    #[anchor2Loc.x, anchor2Loc.y],
    #                                    #[anchor3Loc.x, anchor3Loc.y]
    #                                 ]))

    ## TODO GET ANCHOR POSITIONS CORRECTLY - THESE ARE POSITIONS FROM SLC BALLROOM TEST
    # pf = UWBParticleFilter(anchor_positions = np.array([
    #                                    [0.0, 0.0],
    #                                    [2.43, 0.0],
    #                                    [5.18, 0.0],
    #                                    [-6.09, 8.22],
    #                                    [0.0, 0.0],
    #                                    [0.0, 0.0],
    #                                    [0.0, 0.0],
    #                                    [0.0, 0.0],
    #                                    [0.0, 0.0]
    #                                 ]),
    #                                 tag_transforms = np.array([
    #                                    [0, 0.145],
    #                                    [0.145, 0],
    #                                    [-0.145, 0]
    #                                 ]),
    #                                 initial_position = [0., 0.0],
    #                                 update_rate = UPDATE_RATE)
    #TODO INTRODUCE THE CORRECT LASER TRANSFORM
    pf = UWBParticleFilter(num_sensors = num_sensors, anchor_positions = np.array([
                                       [0.0, 0.0],
                                       [18.28, -2.28],
                                       [18.28, 7.01],
                                       [0.00, 4.62],
                                       [0.0, 0.0],
                                       [0.0, 0.0],
                                       [0.0, 0.0],
                                       [0.0, 0.0],
                                       [0.0, 0.0]
                                    ]),
                                    tag_transforms = np.array([
                                       [0, 0.145],
                                       [0.145, 0],
                                       [-0.145, 0]
                                    ]),
                                    laser_transform = np.array([0, .145]),
                                    initial_position = [0., 0.0],
                                    occ_grid = occupancy_grid,
                                    update_rate = UPDATE_RATE)


    # Create numpy array and mask to store incoming sensor measurement
    sensor_measurements = np.zeros(shape = (num_sensors), dtype = np.float32)
    sensor_measurements_mask = np.ones(shape = (num_sensors))

    ######################################################## SENSOR SUBSCRIBERS ########################################################
    # Set up subscribers for sensor messages
    anchor_distance_subs = [
        rospy.Subscriber("/uwb/0/anchors/9205/distance", Float64, create_uwb_sensor_update_function(0, 0, 0, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/C518/distance", Float64, create_uwb_sensor_update_function(0, 0, 1, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/9AAB/distance", Float64, create_uwb_sensor_update_function(0, 0, 2, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/D81B/distance", Float64, create_uwb_sensor_update_function(0, 0, 3, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/998D/distance", Float64, create_uwb_sensor_update_function(0, 0, 4, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/D499/distance", Float64, create_uwb_sensor_update_function(0, 0, 5, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/9BAC/distance", Float64, create_uwb_sensor_update_function(0, 0, 6, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/1C31/distance", Float64, create_uwb_sensor_update_function(0, 0, 7, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/0/anchors/91BA/distance", Float64, create_uwb_sensor_update_function(0, 0, 8, num_sensors, num_tags, num_anchors, pf)),

        rospy.Subscriber("/uwb/1/anchors/9205/distance", Float64, create_uwb_sensor_update_function(0, 1, 0, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/C518/distance", Float64, create_uwb_sensor_update_function(0, 1, 1, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/9AAB/distance", Float64, create_uwb_sensor_update_function(0, 1, 2, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/D81B/distance", Float64, create_uwb_sensor_update_function(0, 1, 3, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/998D/distance", Float64, create_uwb_sensor_update_function(0, 1, 4, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/D499/distance", Float64, create_uwb_sensor_update_function(0, 1, 5, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/9BAC/distance", Float64, create_uwb_sensor_update_function(0, 1, 6, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/1C31/distance", Float64, create_uwb_sensor_update_function(0, 1, 7, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/1/anchors/91BA/distance", Float64, create_uwb_sensor_update_function(0, 1, 8, num_sensors, num_tags, num_anchors, pf)),

        rospy.Subscriber("/uwb/2/anchors/9205/distance", Float64, create_uwb_sensor_update_function(0, 2, 0, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/9AAB/distance", Float64, create_uwb_sensor_update_function(0, 2, 1, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/C518/distance", Float64, create_uwb_sensor_update_function(0, 2, 2, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/D81B/distance", Float64, create_uwb_sensor_update_function(0, 2, 3, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/998D/distance", Float64, create_uwb_sensor_update_function(0, 2, 4, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/D499/distance", Float64, create_uwb_sensor_update_function(0, 2, 5, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/9BAC/distance", Float64, create_uwb_sensor_update_function(0, 2, 6, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/1C31/distance", Float64, create_uwb_sensor_update_function(0, 2, 7, num_sensors, num_tags, num_anchors, pf)),
        rospy.Subscriber("/uwb/2/anchors/91BA/distance", Float64, create_uwb_sensor_update_function(0, 2, 8, num_sensors, num_tags, num_anchors, pf))
    ]
    # Subscriber for IMU
    imu_sub = rospy.Subscriber("/navx_node/Imu", Imu, create_imu_sensor_update_function(-1))
    # Subsriber for LIDAR
    lidar_sub = rospy.Subscriber("/scan", LaserScan, create_lidar_sensor_update_function(num_tags*num_anchors))

    # Create publisher to publish particle states
    particles_pub = rospy.Publisher('/uwb/pf/particles', PoseArray, queue_size=10)
    # Create publish to publish expected pose
    pose_pub = rospy.Publisher('/uwb/pf/pose', PoseStamped, queue_size = 10)

    # Transform broadcaster to broadcast localization info
    transform_broadcaster = tf.TransformBroadcaster()
    # Transform listener is used to listen to the transform from odom --> base_link frame
    transform_listener = tf.TransformListener()


    rospy.loginfo("Beginning Particle Filtering")

    # Publish at 20 Hz
    rate = rospy.Rate(UPDATE_RATE) # 20hz

    seq = 0

    while not rospy.is_shutdown():
        start = time.clock()
        test = np.ma.masked_array(sensor_measurements, mask = sensor_measurements_mask)
        print("Outside loop")
        print(test)
        pf.update(np.ma.masked_array(sensor_measurements, mask = sensor_measurements_mask))
        # print("Loop time: {}".format(time.clock()-start))

        sensor_measurements_mask = np.ones(shape = (num_sensors))

        # Retrieve particle states
        particles = pf.get_particle_states()
        estimated_pose = pf.get_state()

        # Create message for outputting particle states
        pose_array = PoseArray()

        # Instantiate a list to store all of the poses in the pose message
        pose_list = [None] * particles.shape[0]

        # Iterate over all poses
        for i in range(particles.shape[0]):
            pose_list[i] = Pose()
            # Write position to message
            pose_list[i].position.x = particles[i][0]
            pose_list[i].position.y = particles[i][1]
            pose_list[i].position.z = 0

            # Write orientation to message
            pose_list[i].orientation.x = 0
            pose_list[i].orientation.y = 0
            pose_list[i].orientation.z = np.sin(particles[i][2]/2)
            pose_list[i].orientation.w = np.cos(particles[i][2]/2)

        # Update pose array message
        pose_array.poses = pose_list
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"
        pose_array.header.seq = seq
        seq += 1

        particles_pub.publish(pose_array)

        ### Publish estimated pose as message ###
        pose_msg = PoseStamped()

        # Write position to message
        pose_msg.pose.position.x = estimated_pose[0]
        pose_msg.pose.position.y = estimated_pose[1]
        pose_msg.pose.position.z = 0

        # Write orientation to message
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = np.sin(estimated_pose[2]/2)
        pose_msg.pose.orientation.w = np.cos(estimated_pose[2]/2)

        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.header.seq = seq

        pose_pub.publish(pose_msg)


        ### Publish estimated pose as transform ###
        # According to ROS standards, the localization software onboard the robot should publish the transform from map --> odom frames
        # odom --> base_link represents the estimate of the position of the robot using only odometry data, provides an estimate of pose that is continuous (does not jump)
        #           but that drifts over time
        # Therefore, the localization node publishes the transformation from the map --> odom frame to correct for the drift such that the map --> base_link transformation
        # provides an accurate estimation of the robot pose


        try:
            # First we need to get the estimate of the robot pose from the odometry data
            (trans_base_odom, rot_base_odom) = transform_listener.lookupTransform('/odom', '/base_link', rospy.Time(0))

            # Extract rotation from quaternion
            (roll_base_odom, pitch_base_odom, yaw_base_odom) = tf_conversions.transformations.euler_from_quaternion(rot_base_odom)

            # theta_odom_map is the error between the particle filter estimated orientation vs the odometry estimated orientation
            # Once again assumes 2D robot - robot does not fly or roll
            theta_odom_map = estimated_pose[2] - yaw_base_odom

            #Next, we find the difference in the position estimate between the particle filter and the odometry
            x_odom_map = estimated_pose[0] - trans_base_odom[0]*np.cos(theta_odom_map) + trans_base_odom[1]*np.sin(theta_odom_map)
            y_odom_map = estimated_pose[1] - trans_base_odom[1]*np.cos(theta_odom_map) - trans_base_odom[0]*np.sin(theta_odom_map)
            z_odom_map = 0 # Assuming the robot is not magically flying



            # Finally, we convert back to quaternion
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, theta_odom_map)

            # Publish transform
            transform_broadcaster.sendTransform((x_odom_map, y_odom_map, z_odom_map), q, rospy.Time.now(), "/odom", "/map")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Unable to publish transformation from /map --> /odom because of an error with transform from /odom --> /base_link")
            continue

        if LOG_DATA:
            msg = "{},{}\n".format(estimated_pose[0], estimated_pose[1])
            f.write(msg)


        rate.sleep()

    if LOG_DATA:
        f.close()





