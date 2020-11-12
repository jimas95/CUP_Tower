# final-project-fast-tower
The goal of this project is to use the robot baxter and build a **HUGE** tower from plastic cups

# Run Baxter Simulator
1. cd rethink_ws
2. source devel/setup.bash 
3. cd our project 
4. source devel/setup.bash 
2. roslaunch tower baxter_world.launch

## control baxter and Rviz with moveit
1. roslaunch tower baxter_world.launch
2. rosrun baxter_interface joint_trajectory_action_server.py
3. rosrun baxter_interface gripper_action_server.py
4. rosrun baxter_tools enable_robot.py -e
5. roslaunch baxter_moveit_config demo_baxter.launch right_electric_gripper:=True left_electric_gripper:=True
6. rosrun tower control_arm joint_states:=robot/joint_states

note: remember to change `joint_limits.yaml` to values specified in `joint_limits_true.yaml` for real robot.

## calling tests service with the following choices
0. print current pose of left and right arm
1. test step service with left arm 
2. test step service with right arm 

# Apriltag Install
`pip3 install apriltag`


# Usefull links 
## from matt
https://nu-msr.github.io/me495_site/final_project2020.html
https://nu-msr.github.io/me495_site/lecture13_rethink.html

## Baxter 
1. official page: https://sdk.rethinkrobotics.com/wiki/Home
2. git : https://github.com/RethinkRobotics/

## moveIT
https://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial
https://github.com/ros-planning/moveit_robots/tree/kinetic-devel/baxter/baxter_moveit_config/config


