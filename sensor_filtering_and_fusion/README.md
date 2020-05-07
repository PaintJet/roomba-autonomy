# sensor_filtering_and_fusion

This package contains all of the scripts to ingest sensor data and generate meaningful outputs. The two scripts included are the particle_filter and lidar_angle_filter.

## Particle Filter
Particle filter is the main script for localization. A particle filter a sample-based probabilistic filtering algorithm that is used to generate an estimate of the pose of the robot using all of the incoming sensor data. Since it is sample-based, it can estimate arbitrary distributions of potential states. There a couple of main portions of the particle filter that we have implemented.

The particle filter works by maintaining a set of "particles", which are estimates of the robot state. At each time step, the likelihood of the particle being the actual state of the robot is calculated. The low likelihood parameters are eliminated and new particles are sampled from the distribution created by the high likelihood particles. Noise is added at each step.

Parameters:

* n - number of particles
* d - dimension of state
* h - dimension of sensor measurements

### State
The state of the robot is all of the parameters that describe the current configuration of the robot. Since we are using the particle filter for pose estimation, our state is a 3x1 vector: (x,y,theta)

### Motion model/dynamics_function
The motion model describes how the robot state evolves based on the control input to the robot. The motion model accepts the linear and angular velocity of the robot (Twist) and the update rate of the filter. It outputs the updated state of each particle with the applied motion.

Arguments:

* particles_states - (n,d) vector containing the state of all of the particles

Returns: (nxd) vector containing updated particle states

### Sensor Model
The sensor model processes the sensor data and accordingly updates the particle filter. This model has two parts.
#### Observation Function
We define an observation function for each sensor. For each particle, this function will calculate the sensor measurement we expect to receive if the particle state is the true state of the robot.

Arguments:

* particles_states - (n,d) vector containing the state of all of the particles

Returns: (nxh) vector containing the expected sensor measurements for each state

* UWB: Distance to each anchor
* IMU: Orientation
* Lidar: Pose of the lidar - this weird sensor measurement has to do with the way we calculate the particle weight using lidar


#### Weighting Function
The weighting function accepts the sensor measurement from the actual sensors and the hypothetical sensor measurements for each particle. We find the error between the hypothetical and actual sensor measurements. Based on the noise characteristics of the sensors and the error, we assign a probability to each particle describing how likely it is to describe the correct state.

Arguments:

* hyp_observed - (n,h) hypothetical observations for each particle
* real_observed - (h,) real observations

Returns: (n,) vector containing weights for each particle

### Known Issues:

* Particle filter is not stable for long run durations, it crashes usually after 10 minutes due to NaN errors
* The motion model does not include a noise term to account for the uncertainty created from the motion
* Wheel odometry is not fused into the state estimate
* Coordinate frame for robot and world do not align - this results in a extra pi/2 in the dynamics function that should not exist
* The parameters for determining the particle weights are NOT tuned
* The ROS nodes need to be launched in a specific order to correctly launch the particle filter, this order dependence needs to be eliminated

## lidar_angle_filter
This launch file instantiates a ROS node for filtering a certain angle range out of the LIDAR data. This is because half the sensor is facing the robot, so it is useless data. It uses the `scan_to_scan_filter_chain` from the `laser_filters` package

### Known Issues
* Using this configuration, we cannot filter the angle such that the wire from the LIDAR is facing the robot. Currently, the wire has to be coming out of the side of the sensor.
