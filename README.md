#  1st ASSIGNMENT of Research Track 2 


This project is a ROS 2 application that lets a user send navigation goals to a robot through an action interface. The physic simulator used is Gazebo, thus, to correctly use the application, it's necessary to install the github repository https://github.com/CarmineD8/bme_gazebo_sensors.
The user can:

- send a new goal (x, y, theta)
- cancel the currently running goal
- quit the application

The architecture is split into two packages:

- assignment_1_rt2: application nodes (user interface, action client, action server, TF broadcasters, launch file)
- assignment_1_rt2_interfaces: custom ROS 2 interfaces (message + action)

## Package Structure

### assignment_1_rt2

Main logic and runtime nodes:

- src/user_interface.cpp
- src/action_client.cpp
- src/action_server.cpp
- src/broadcaster.cpp
- launch/launcher.launch.py

### assignment_1_rt2_interfaces

Custom interfaces:

- msg/UserMsg.msg
- action/Navigation.action

## Nodes and Responsibilities

### user_interface (standalone node)

Reads terminal input and publishes commands on topic user_msg:

- g: send a navigation goal (x, y, theta)
- c: cancel current goal
- q: quit

### NavigationActionClient (component)

- Subscribes to user_msg
- Sends goals to action server navigation
- Cancels active goals when requested
- Publishes goal pose as nav_msgs/Odometry on topic goal_frame
- Logs action feedback and final result

### NavigationActionServer (component)

- Provides action server navigation
- Subscribes to odom for robot state
- Publishes cmd_vel commands to move the robot in three stages:
  1. rotate toward target position
  2. move toward target position
  3. rotate to target orientation
- Handles cancel requests and returns remaining errors in action result

### FramePublisher (component, instantiated twice)

Generic TF broadcaster node parameterized by topic_name:

- topic_name=goal_frame: broadcasts goal transform
- topic_name=odom: broadcasts robot transform

## Interfaces

### assignment_1_rt2_interfaces/msg/UserMsg.msg

- float32 x_pos
- float32 y_pos
- float32 theta
- string msg

### assignment_1_rt2_interfaces/action/Navigation.action

Goal:

- float32 goal_x
- float32 goal_y
- float32 goal_theta

Result:

- float32 delta_x
- float32 delta_y
- float32 delta_theta

Feedback:

- float32 remaining_x
- float32 remaining_y
- float32 remaining_theta

## Topics and Action

### Topics

- user_msg (assignment_1_rt2_interfaces/msg/UserMsg): commands from UI to action client
- goal_frame (nav_msgs/msg/Odometry): goal pose published by action client
- odom (nav_msgs/msg/Odometry): robot odometry consumed by action server and TF broadcaster
- cmd_vel (geometry_msgs/msg/Twist): velocity commands published by action server

### Action

- navigation (assignment_1_rt2_interfaces/action/Navigation)

## Launch

The launch file starts:

- user_interface as a regular node
- a component container with:
  - NavigationActionServer
  - NavigationActionClient
  - FramePublisher for goal_frame
  - FramePublisher for odom

File path:

- assignment_1_rt2/launch/launcher.launch.py

## Requirements

- ROS 2 Jazzy(tested in a ROS 2 workspace)
- colcon
- an external simulator or robot stack that provides odom and consumes cmd_vel
- xterm (used by launch prefixes in launcher.launch.py)

## Build and Run

From your ROS 2 workspace root (for example ~/Documents/ros2_ws):

1. Build:

   colcon build

2. Source the workspace in 2 different terminals:

   cd install/
   source local_setup.bash
   
3. Launch Gazebo in the first terminal:

    ros2 launch bme_gazebo_sensors spawn_robot_ex.launch.py

3. Launch the system in the second terminal:

   ros2 launch assignment_1_rt2 launcher.launch.py

## How to Use

When prompted by the user interface, enter one of the following:

- x y theta (example: 2.0 1.5 1.57) to send a new goal
- c to cancel the current goal
- q to quit

Behavior notes:

- Sending a new goal while one is active triggers a cancel of the current goal, then sends the new goal.
- Cancel and quit requests are forwarded by the action client to the action server.

## Author

Daneri Gregorio
