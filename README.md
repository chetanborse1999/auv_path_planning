# Path Planning Of AUV Using ROS/Gazebo/UUV_Simulator
This page briefly describes the implementation of Path Planning for AUV in GPS denied environment. The project is a Gazebo Simulation submitted in partial fulfillment of course requirements for (MAE598 Intro to Autonomous Vehicles). Refer to project report for theoretical concepts and implementation details.

<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/simulation.gif?raw=true" width="400"></p>

## Objective

In underwater environments, path planning is a challenging due to it being a GPS-denied environment and limited visibility. To that end, we devise an algorithm that uses the onboard suit of sensors to detect obstables and plan a shortest path through the unexplored domain by method that is essentially dead reckoning. We use the onboard IMU for AUV orientation, DVL sensor for velocity and a LiDAR for obstacle detection. The sensor data is processed and a shortest path to the target position is generated using the A* search algorithm. A map of detected obstacles is stored that keeps track of obstacles and trajectory is recalculated when the way is blocked. A cascaded PID controller then takes the path and drives the thrusters such that the AUV follows the trajectory. The project is built as a ROS package on top of the UUV Simulator environement built by M. Manhães for the EU SWARMS initiative.

## Methodology

<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/modules_flowchart.png?raw=true" width="640"></p>

The system is made of separate modules each responsible for a critical aspect. IMU and DVL make up the Odometry module. LiDAR is the module for obstacle detection. Occupancy Grid module combines the odometry data with the Lidar data to create an occupancy grid map that tracks the position of obstacles. Path Planning module uses this map and odometry data to plan the shortest path to target using A* search. And lastly the controller module takes the trajectory from the Path planning module to guide the AUV along the path to target

### Odometry

<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/transform_to_world_frame.png?raw=true" width="640"></p>

The velocity from DVL sensor and orientation from IMU is combined to give the relative position of the AUV from its initial position. A coordinate transform is done to convert the position from robot frame to world frame.

### Obstacle detection

<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/lidar_scan_to_map_transform.png?raw=true" width="640"></p>

The data from LiDAR scan is in form of an array size of which is depends of the discretization of the angle of scan and the data in the array is the range of obstacle from the robot. The obstacle positions from data is then converted to robot frame and further converted to world frame by coordinate transformations.

### Path Planning

<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/a_star_simple_sim.png?raw=true" width="480"></p>

The system uses A* for planning shortest path. We tested the algorithm in a simpler 2D discrete space as shown before adapting it to continuous world space in Gazebo.

### ROS Package

A ROS package was made that combines all modules in a single folder and can be launched from a single ROS Launch file. 

<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/file_structure.png?raw=true" width="480"></p>

## Results

<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/simulation_timestep_1_2.png?raw=true" width="640"></p>
The following simulation was captured in the Gazebo from Top View showing the AUV plan around column obstacles towards the target position.
<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/simulation_timestep_3_4.png?raw=true" width="640"></p>
<p align="center"><img src="https://github.com/chetanborse1999/auv_path_planning/blob/main/simulation_final.png?raw=true" width="320">
<br>The AUV reaches the target position autonomously.</p>
