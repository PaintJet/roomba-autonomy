# roomba-autonomy
This is the main repository for codebase of the PaintJet autonomous painting robot project. The objective of the project is to design a modular robotic platform for high-volume, low-quality painting of large buildings like warehouses. The initial iteration of the robot will be designed to apply epoxy to the floor of these warehouses. The software is first being developed for a Roomba Create 2, a prebuilt robotic platform that allows us to quickly test and iterate on our code.

This codebase includes software for autonomous localization, mapping, and navigation for the robot as well as the controls for applying epoxy to the floor. Each of the subfolders are ROS packages containing code for the various functions of the robot. A brief explanation of each packages is presented below in Packages and the details of each package are provided in the ReadMe files in each package.

## Installation and Setup
This guide assumes that you have already installed ROS Melodic (for Ubuntu 18.04). Navigate to the `src` directory inside your `catkin_ws` and clone this repository.

```
cd ~/catkin_ws/src
git clone https://github.com/PaintJet/roomba-autonomy.git
```

Navigate back to the top directory of the `catkin_ws` and run `catkin_make` 
```
cd ~/catkin_ws
catkin_make
```

## Packages

### aruco_localization
Software for determining the location of a camera relative to an ArUco marker placed in the environment. This package was developed for testing the performance of this localization method, but is no longer being used for localization due to poor performance at long distance and large angles.

### teleop_control
Allows the robot to be controlled using a joystick. This is used for testing purposes of the motor control code.

### uwb_localization
Localization of the robot using the Decawave DWM1001 ultra-wide band modules. This is the method the robot is currently using for localization.
