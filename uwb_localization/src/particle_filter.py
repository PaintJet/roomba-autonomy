#!/usr/bin/env python

from __future__ import print_function

# ROS imports
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray
from std_msgs.msg import Float64


from pfilter import ParticleFilter, independent_sample, squared_error, gaussian_noise
import numpy as np
from scipy.stats import uniform




class UWBParticleFilter:
    # Implementation of a particle filter to predict the pose of the robot given distance measurements from the modules
    def __init__(self, anchor_positions):
        # Members of the state
        self.state_desc = ["x", "y"]

        # Prior sampling function for each of the state variables
        # We assume uniform distribution over [0 m, 20 m]
        self.prior_fn = independent_sample([
            uniform(loc = -10, scale = 50).rvs,
            uniform(loc = -10, scale = 50).rvs
        ])

        self.anchor_positions = anchor_positions

        self.pf = ParticleFilter(
            n_particles = 1000,
            prior_fn = self.prior_fn,
            observe_fn = self.observation_function,
            weight_fn = lambda x,y:squared_error(x, y, sigma=0.6),
            noise_fn = lambda x: gaussian_noise(x, sigmas=[0.2, 0.2])
        )

        self.pf.update()

    # observation_function
    # Accepts the state of all the particles and returns the predicted observation for each one
    # particles states - n x d array: n is the number of particles and d is state dimension
    def observation_function(self, particle_states):
        # Create output array of dimension n x h
        # h is the dimension of the observation
        expected_observations = np.zeros(shape = (particle_states.shape[0], self.anchor_positions.shape[0]))

        # TODO Make this more efficient using map or something
        # Calculated expected observations
        for i in range(particle_states.shape[0]):
            for j in range(self.anchor_positions.shape[0]):
                #print("{} {}".format(particle_states[i], self.anchor_positions[j]), end = "\n")

                # Calculate distance between ith particle and jth anchor
                expected_observations[i][j] = np.linalg.norm(particle_states[i] - self.anchor_positions[j])
        return expected_observations

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

# create_observation_function
# Factory function that generates an observation function for each of the UWB anchors
# The observation function will craft the observation input into the particle filter and feed into the pf
def create_observation_function(anchor_number, num_anchors, pf):
    def update_pf(message):
        # Create array for particle filter observation
        pf_input = np.zeros(shape = (num_anchors), dtype = np.float64)

        # Create array to mask observation matrix
        mask = np.ones(shape = (num_anchors))

        # Write the distance observation to the correct index associated with the anchor
        pf_input[anchor_number] = message.data
        # Mask all elements except anchor_number
        mask[anchor_number] = 0

        pf.update(np.ma.masked_array(pf_input, mask = mask))

    return update_pf




if __name__ == "__main__":
    ## Variables for data logging
    # Flag to decide if data should be logged during run
    LOG_DATA = True
    log_path = "uwb-data.csv"

    # Open file to store logs
    if LOG_DATA:
        f = open(log_path, "w")

    # Init ros node
    rospy.init_node("uwb_particle_filter")

    rospy.loginfo("Initializing Particle Filter Node...")

    # Read the locations of the anchors from the ros messages
    anchor0Loc = rospy.wait_for_message('/uwb/0/anchors/9205/position', Point)
    # anchor1Loc = rospy.wait_for_message('/uwb/0/anchors/9AAB/position', Point)
    # anchor2Loc = rospy.wait_for_message('/uwb/0/anchors/C518/position', Point)
    # anchor3Loc = rospy.wait_for_message('/uwb/0/anchors/D81B/position', Point)

    # Instantiate particle filter
    # pf = UWBParticleFilter(np.array([[anchor0Loc.x, anchor0Loc.y]
    #                                    #[anchor1Loc.x, anchor1Loc.y],
    #                                    #[anchor2Loc.x, anchor2Loc.y],
    #                                    #[anchor3Loc.x, anchor3Loc.y]
    #                                 ]))
    pf = UWBParticleFilter(np.array([[0.0, 0.0],
                                       [9.14, 13.11],
                                       [0.0, 13.41],
                                       [7.01, 1.22]
                                    ]))

    # Set up subscribers for sensor messages
    anchor_distance_subs = [
        rospy.Subscriber("/uwb/0/anchors/9205/distance", Float64, create_observation_function(0, 4, pf)),
        rospy.Subscriber("/uwb/0/anchors/9AAB/distance", Float64, create_observation_function(1, 4, pf)),
        rospy.Subscriber("/uwb/0/anchors/C518/distance", Float64, create_observation_function(2, 4, pf)),
        rospy.Subscriber("/uwb/0/anchors/D81B/distance", Float64, create_observation_function(3, 4, pf))
    ]

    # Create publisher to publish particle states
    particles_pub = rospy.Publisher('/uwb/pf/particles', PoseArray, queue_size=10)
    # Create publish to publish expected pose
    pose_pub = rospy.Publisher('/uwb/pf/pose', PoseStamped, queue_size = 10)


    rospy.loginfo("Beginning Particle Filtering")
    # Publish at 10 Hz
    rate = rospy.Rate(100) # 10hz
    seq = 0
    while not rospy.is_shutdown():
        # Retrieve particle states
        particles = pf.get_particle_states()
        estimated_pose = pf.get_state()

        # Create message
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
            pose_list[i].orientation.z = 0
            pose_list[i].orientation.w = 1

        # Update pose array message
        pose_array.poses = pose_list
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "map"
        pose_array.header.seq = seq
        seq += 1

        particles_pub.publish(pose_array)

        ### Publish estimated pose ###
        pose_msg = PoseStamped()
        # Write position to message
        pose_msg.pose.position.x = estimated_pose[0]
        pose_msg.pose.position.y = estimated_pose[1]
        pose_msg.pose.position.z = 0

        # Write orientation to message
        pose_msg.pose.orientation.x = 0
        pose_msg.pose.orientation.y = 0
        pose_msg.pose.orientation.z = 0
        pose_msg.pose.orientation.w = 1

        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        pose_msg.header.seq = seq

        pose_pub.publish(pose_msg)

        if LOG_DATA:
            msg = "{},{}\n".format(estimated_pose[0], estimated_pose[1])
            f.write(msg)


        rate.sleep()

    if LOG_DATA:
        f.close()





