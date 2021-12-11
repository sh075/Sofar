# Sofar
### Software Architecture Project


## Student: Shahrzad Eskandari Majdar (S5060737)

## Tutor(s): Fulvio Mastrogiovanni , mohamad Alameh, Alessandro Carfi, Simone Macciٍ


This project is a simulation of a mobile robot that navigates autonomously in a specified environment. In this case, the robot (husky) is outfitted with a lidar sensor, a microprocessor, and a four-wheel drive system. The robot is supposed to navigate autonomously while mapping its surroundings and avoiding obstacles until it arrives at its goal.

The main goal of the research is to create a safe path for the robot to follow by processing data from sensors such as odometry and lidar in the environment map. We construct environment maps, localize the robot in the environment, have the robots plan their paths, view data from the various Navigation processes and employing SLAM, and configure the various Navigation nodes.


## uml diagram
![uml diagram](https://github.com/sh075/Sofar/blob/main/uml%20diagram.png)


## sequential diagram
![sequentional](https://github.com/sh075/Sofar/blob/main/sequentional.png)
## modules that are used

# slam-gmapping

The gmapping package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map from laser and pose data collected by a mobile robot.

make sure you install the pacage in the current working directory 

`https://github.com/ros-perception/slam_gmapping.git`

In order to establish a connection of the Lidar to ROS Kinetic

`sudo apt-get install ros-noetic-lms1xx`

# Rviz

rviz is a 3d visualization tool for ROS applications. It provides a view of your robot model, captures sensor information from robot sensors, and replay captured data. It can display data from cameras, lasers, from 3D and 2D devices including pictures and point clouds.The robot state publisher helps you to broadcast the state of your robot to the tf transform library.Joint state publisher is one of the ROS packages that is commonly used to interact with each joint of the robot. The package contains the joint_state_publisher node, which will find the nonfixed joints from the URDF model and publish the joint state values of each joint in the sensor_msgs/JointState message format.

To install rviz in ros 

`sudo apt-get install -y rviz`

For installing joint state publisher

`sudo apt-get install -y joint-state-publisher`

For installing robot state publisher

`sudo apt-get install ros-noetic-robot-state-publisher`


# move_base
The move_base node provides a ROS interface for configuring, running, and interacting with the navigation stack on a robot .Running the move_base node on a robot that is properly configured results in a robot that will attempt to achieve a goal pose with its base to within a user-specified tolerance. In the absence of dynamic obstacles,

Packages that need to be installed

`cd /opt/ros/noetic/lib`

`sudo apt-get install ros-noetic-move-base-msgs`


# Planner

This shows the local costmap that the navigation stack uses for navigation. The yellow line is the detected obstacle. For the robot to avoid collision, the robot's footprint should never intersect with a cell that contains an obstacle.
In the global costmap is everything the robot knows from previous visits and stored knowledge e.g. the map. In the local costmap is everything that can be known from the current position with the sensors right now. 

# Navigation

The Navigation Stack is fairly simple on a conceptual level. It takes in information from odometry and sensor streams and outputs velocity commands to send to a mobile base. As a prerequisite for navigation stack the robot should have a tf transform tree in place, and publish sensor data using the correct ROS Message types. Also, the Navigation Stack needs to be configured for the shape and dynamics of a robot to perform at a high level. To help with this process, this manual is meant to serve as a guide to typical Navigation Stack set-up and configuration.


To install the package

`sudo apt-get install ros-noetic-navigation`



# Execution of the code

Git clone the package

`https://github.com/sh075/Sofar`


To launch the handshake between unity and ros

`roslaunch mobile_robot_navigation_project nav.launch`

for localisation the robot in the environment

`roslaunch mobile_robot_navigation_project gmapping.launch`

for urdf of husky etc in rviz

`roslaunch mobile_robot_navigation_project rviz.launch`

To plan the path and navigate the robot 

`roslaunch mobile_robot_navigation_project move2.launch`

for rqt graph

`rosrun rqt_graph rqt_graph`
![rosgraph](https://github.com/sh075/Sofar/blob/main/rqt_graph.png)






