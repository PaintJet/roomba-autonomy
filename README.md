# roomba-autonomy
This is the main repository for codebase of the PaintJet autonomous painting robot project. The objective of the project is to design a modular robotic platform for high-volume, low-quality painting of large buildings like warehouses. The initial iteration of the robot will be designed to apply epoxy to the floor of these warehouses.

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

#
