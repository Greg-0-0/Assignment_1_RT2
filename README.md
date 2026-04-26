#  1st ASSIGNMENT of Research Track 2 


This project consists in a ros2 application that allows to control a planar drone in a 3D environment. To move the drone, the user has to insert an angular and a linear velocity, then the application applies these values to the drone, moving it for one second. The application uses a simulator called Gazebo to represent the 3D space, while the model of the robot is provided by the tool Rviz. Moreover, the Rviz framework allows to identify eventual obstacles inside the environmnet, making possible to avoid them while guiding the drone. Indeed, if the drone gets closer than a certain threshold to an obstacle, after the second of movement elapses, its motion gets reversed until it reaches the safe area, farther away than the threshold. However, the drone gets stopped during motion if it gets too close to an obstacle (beyond the threshold), surpassing a second limit, this prevents the drone from colliding with obstacles. The threhsold can be changed at runtime thanks to a specific service, while the smaller limit is fixed. The application also provides the possibility to compute the average of the last five velocities inserted by the user, as well as obtaining information relative to the closest obstacle to the drone, like its relative direction and distance from the drone.

# Packages
- assignmnet2_rt:                main package where the movement logic and the obstacle avoidance are implemented
- assignment2_rt_interfaces:     provides the services to update the threshold, compute the averages and the custom data structure for the drone information
- assignment2_rt_bringup:        allows to conveniently start the application without running the single executables separately
- bme_gazebo_sensors:            implements the 3D envirnoment and the Rviz tool for the drone representation [not implemented in this project, but necessary for the functioning of the application]

# Executables
assingmnet2_rt:
- ui_node ->            manages the user input moving the drone, and prevents eventual collisions with the obstacles using the information provided by the distance_node
- distance_node ->      computes the collision condition using the provided threshold, and shares with the ui_node the information of the closest obstacle
assinbgmnet2_rt_interfaces:
- threshol_change_service ->     handles the request to update the threshold
- velocity_averages_service ->   handles the request to compute the velocities average
assignment2_rt_bringup:
- launcher.launch.py ->   starts all the executables from a single command

# Topics
Default topics:
- /scan (provided by Gazebo)
- /cmd-vel (provided by Gazebo)

Custom topics:
- collision_condition: used by the distance_node to inform the ui_node if the drone overcame the threshold or the smaller limit
- obstacle_info: used by the distance_node to send the information of the closest obstacle to the ui_node
- update_threhsold: used by the corresponding service to notify the distance_node of the update

# Interfaces
- update_threhsold: service to change the threshold value upon request of the ui_node
- velocity_averages: service to compute the velocities average upon request of the ui_node
- ObstacleInfo:  custom message used by the distance_node to represent the information of the closest obstacle and share them with the ui_node

# Requirements
- ros2 framework

# Instructions to run
First download the files and directories from GitHub into your ros2 workspace.  
Then in a ros2 terminal run:
1. colcon build ->                      executed in workspace directory to compile the package
2. source local_setup.bash ->           run inside install/ folder
3. ros2 launch assingment2_rt_bringup launcher.launch.py ->  to execute the application (only one terminal necessary)

# Instructions to use
Initially, the application waits a couple of seconds the ensure all the topics are correctly communicating.  
Follow the textual interface selecting an option:

1 move the drone:
  - insert an angular velocity
  - insert a linear velocity
2. update threshold  
3. compute velocity averages  
4. obtain closest obstacle information  

# Functions
Distance.cpp:
- topic_callback1: retrieves the obstacles distances from the topic "/scan"
- topic_callback2: retrieves the newly updated threhdold from the topic "obstacle_info"
- main_loop: computes distance between two turtles, publishing them to ui_node, and prints info on closest obstacle

UI.cpp:
- initialization: delays the start of the applicatio to ensure all topics are properly working
- main_loop: implements the main function from which all the other functions gets called directly or indirectly
- user_input: displays textual interface to choose an option
- rotate_turtle: applies angular velocity for one second (stopped by timer_callback1)
- timer_callback1: stops the rotation after one second (called by rotate_turtle)
- threhdold_request: sends request to threshold_change_service for new threshold value
- velocity_averages_request: sends request to velocity_averages_service for velocity averages computation
- move_turtle_x:  moves the drone for one second (stopped by timer_callback2)
- timer_callback2: stops the translation (calls stop_linear_motion) after one second (called by move_turtle_x) and eventually applies a revers movement if the threshold has been overcame (exeuted by start_backward_motion)
- start_backward_motion: starts backward motion after robot has stopped (called by timer_callback2) (stopped by timer_callback3)
- timer_callback3: stops turtle translation for position restoration
- topic_callback1: receives condition relative to distance between the drone and obstacles in real time from the topic "collisin_condition"
- topic_callback2: receives obstacle info from Distance node from the topic "obstacle_info"

threhsold_update_service:
- handle_service: handles the request to update the threshold

velocity_averages_service:
- handle_service: handles the request to compute the velocities average
  
# Author
Daneri Gregorio

# -----------> Have fun!!! <-----------
