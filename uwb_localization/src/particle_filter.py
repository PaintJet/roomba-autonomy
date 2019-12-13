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
    # anchor_positions is the ground truth location of all of the anchors
    # tag_transforms is a 2d numpy array storing the [x,y] coordinates of each tag in the robot frame
    def __init__(self, anchor_positions, tag_transforms):
        # Number of particles
        self.n = 1000

        # Members of the state
        self.state_desc = ["x", "y", "theta"]

        # Prior sampling function for each of the state variables
        # We assume uniform distribution over [0 m, 20 m]
        self.prior_fn = independent_sample([
            uniform(loc = -50, scale = 50).rvs,
            uniform(loc = -50, scale = 50).rvs,
            uniform(loc =   0, scale = 2*np.pi)
        ])

        self.num_anchors = anchor_positions.shape[0]
        self.anchor_positions = anchor_positions

        self.pf = ParticleFilter(
            n_particles = self.n,
            prior_fn = self.prior_fn,
            observe_fn = self.observation_function,
            weight_fn = lambda x,y:squared_error(x, y, sigma=0.6),
            noise_fn = lambda x: gaussian_noise(x, sigmas=[0.2, 0.2, 0.2])
        )

        self.num_tags = tag_transforms.shape[0]
        self.tag_transforms = tag_transforms

        self.pf.update()

    # observation_function
    # Accepts the state of all the particles and returns the predicted observation for each one
    # particles states - n x d array: n is the number of particles and d is state dimension
    def observation_function(self, particle_states):
        # Create output array of dimension n x h
        # n is the numer of particles
        # h is the dimension of the observation
        expected_observations = np.zeros(shape = (self.n, self.num_tags * self.num_anchors))

        # TODO Make this more efficient using map or something
        # Calculated expected observations
        for i in range(self.n):
            for j in range(self.num_tags):
                for k in range(self.num_anchors):
                    #print("{} {}".format(particle_states[i], self.anchor_positions[j]), end = "\n")

                    # Calculate expected position of the tag in the world frame
                    x_tag = self.tag_transforms[j][0] * np.cos(particle_states[i][2]) - self.tag_transforms[j][1] * np.sin(particle_states[i][2]) + self.anchor_positions[j][0]
                    y_tag = self.tag_transforms[j][0] * np.sin(particle_states[i][2]) + self.tag_transforms[j][1] * np.cos(particle_states[i][2]) + self.anchor_positions[j][1]
                    expected_tag_positon = np.array([x_tag, y_tag])

                    # Expected observation is the
                    expected_observations[i][self.num_anchors * j + k] = np.linalg.norm(expected_tag_positon - self.anchor_positions[j])
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
# tag_number - Which tag on the robot is it
# anchor_number - Which anchor in the environment is it
# num_tags - Total number of UWB tags on the robot
# num_anchors - Total number of anchors in the environment
def create_observation_function(tag_number, anchor_number, num_tags, num_anchors, pf):
    def update_pf(message):
        # Create array for particle filter observation
        pf_input = np.zeros(shape = (num_anchors * num_tags), dtype = np.float64)

        # Create array to mask observation matrix
        mask = np.ones(shape = (num_anchors * num_tags))

        # Write the distance observation to the correct index associated with the tag and anchor
        pf_input[num_anchors*tag_number + anchor_number] = message.data

        # Mask all elements except the one for the measurement we collected
        mask[num_anchors*tag_number + anchor_number] = 0

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
    pf = UWBParticleFilter(anchor_positions = np.array([
                                       [0.0, 0.0],
                                       [9.14, 13.11],
                                       [0.0, 13.41],
                                       [7.01, 1.22]
                                    ]),
                                    tag_transforms = np.array([
                                       [0, 0.145],
                                       [0.145, 0],
                                       [-0.145, 0]
                                    ]))

    # Set up subscribers for sensor messages
    anchor_distance_subs = [
        rospy.Subscriber("/uwb/0/anchors/9205/distance", Float64, create_observation_function(0, 0, 3, 4, pf)),
        rospy.Subscriber("/uwb/0/anchors/9AAB/distance", Float64, create_observation_function(0, 1, 3, 4, pf)),
        rospy.Subscriber("/uwb/0/anchors/C518/distance", Float64, create_observation_function(0, 2, 3, 4, pf)),
        rospy.Subscriber("/uwb/0/anchors/D81B/distance", Float64, create_observation_function(0, 3, 3, 4, pf)),

        rospy.Subscriber("/uwb/1/anchors/9205/distance", Float64, create_observation_function(1, 0, 3, 4, pf)),
        rospy.Subscriber("/uwb/1/anchors/9AAB/distance", Float64, create_observation_function(1, 1, 3, 4, pf)),
        rospy.Subscriber("/uwb/1/anchors/C518/distance", Float64, create_observation_function(1, 2, 3, 4, pf)),
        rospy.Subscriber("/uwb/1/anchors/D81B/distance", Float64, create_observation_function(1, 3, 3, 4, pf)),

        rospy.Subscriber("/uwb/2/anchors/9205/distance", Float64, create_observation_function(2, 0, 3, 4, pf)),
        rospy.Subscriber("/uwb/2/anchors/9AAB/distance", Float64, create_observation_function(2, 1, 3, 4, pf)),
        rospy.Subscriber("/uwb/2/anchors/C518/distance", Float64, create_observation_function(2, 2, 3, 4, pf)),
        rospy.Subscriber("/uwb/2/anchors/D81B/distance", Float64, create_observation_function(2, 3, 3, 4, pf))
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
            pose_list[i].orientation.z = np.sin(particles[i][2]/2)
            pose_list[i].orientation.w = np.cos(particles[i][2]/2)

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
        pose_msg.pose.orientation.z = np.sin(estimated_pose[2]/2)
        pose_msg.pose.orientation.w = np.cos(estimated_pose[2]/2)

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





