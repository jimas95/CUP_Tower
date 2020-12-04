# About
A ROS project developed as part of ME495 - Embedded Systems in Robotics course at Northwestern University.

# Project Description
The goal of this project is to use the BAXTER robot and build a **HUGE** tower from plastic cups

# Quickstart Usage Instructions
1. Set up workspace and clone repository
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/ME495-EmbeddedSystems/final-project-fast-tower 
```
2. Set up rethink workspace
```
mkdir -p rethink_ws/src
cd rethink_ws/src
vcs import --input https://nu-msr.github.io/me495_site/rethink.rosinstall
cd ..
# Install dependencies, but ignore any errors
rosdep install --from-paths src --ignore-src -r -y 
```
3. catkin and source
```
cd rethink_ws
catkin_make
source devel/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
4. Apriltag_ROS Install
```
cd ~/catkin_ws
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone https://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
catkin_make_isolated  
```
5. Setup Baxter Robot
```
`<nmcli con up Rethink>`
`<ping baxter.local>`  does ping ? 
`<export ROS_MASTER_URI=http://10.42.0.2:11311>`
`<export ROS_IP=10.42.0.1>`
`<unset ROS_HOSTNAME>`
`<rosnode list>` should show stuff
`<rostopic echo /robot/joint_states >`
```
6. Enable the robot
```
Enable    :`<rosrun baxter_tools enable_robot.py -e>`
Disable   :`<rosrun baxter_tools enable_robot.py -d>`
Show state:`<rosrun baxter_tools enable_robot.py -e>`
```
8. Control the robot in simulation
```
skip step 5 & 6 
set global variable REAL_ROBOT to False at file arm_control
roslaunch tower baxter_world.launch
roslaunch tower build_tower.launch
rosrun tower arm_control joint_states:=robot/joint_states
```


9. Control the real robot
```
set global variable REAL_ROBOT to True at file arm_control
roslaunch tower build_tower.launch
rosrun tower arm_control joint_states:=robot/joint_states
```

# Example 
![](https://github.com/ME495-EmbeddedSystems/final-project-fast-tower/blob/main/GIFs/3-Cups-build-tower.gif)



# Project Architecture
2 different aproaches where made for this project.
1. Is implemented at arm_control.
2. Is implemented at arm_control above version 2.

The first implementation we use all cameras(3 in total) from Baxter and using computer vision(april tags) we track each position of the cup. Then, the project is split into 2 main tasks.

Task 1: the robot removes all the cups that are in the middle of the table and places them at the left/right side of the table using both hands simultaneously.

Task 2: we scan again and update the new positions of the cups(now located at the sides of table) grab each one of them and build a tower of cups.

This implementation is much more sofisticated and universal. Cups will be picked from "random positions" and the number of cups does not have to be specified (builds tower until there are no cups). Goal positions are calculated in real time (not hard coded) and moveIt scene attaching/detach objects is being used in order to avoid collisions. Different planners are being used to make sure moves will be executed. For example, if cartesian path fails to find path we will retry using a different planner.

The second implementation is much more simple we use hard coded points for grabing and placing each cup in place of computer vision. 

The first implementation did not work consistently, therefore, the second implimentation is primarily used in our project.


# Packages
## 1. tower
The tower package is the primary package used to control the robot and create the planning scene. There are different arm_control node versions each completes various tasks.
### Nodes

1. **arm_control_2_1**: stacks cups concentrically into a stack
2. **arm_control_2_2**: takes cups from middle of the workspace and moves them to the side of the table (cleans the robot workspace) **Task 1**
3. **arm_control_2_3**: builds a 3 cup tower using cartesian coordinates **Task 2**
4. **arm_control_3_1**: uses computer vision (april tags) to locate the initial postion of the cups, then stack the cups into a 3 cup tower **Task 2**
5. **arm_control_3_2**: uses cartesian coordinates and one arm to locate 6 cups and place them at side of the table **Task 1**
6. **arm_control_3_3**: uses computer vision (april tags) to locate the initial position of the cups,using one arm, stack the cups into a 6 cup tower 
7. **arm_control_4_1**: uses cartesian coordinates and both Baxter arms to place 6 cups in a tower **Task 2**
8. **arm_control_4_2**: uses both baxter arms to take 6 cups from middle of the workspace and move them to the side of the table (cleans the robot workspace) **Task 1**
9. **arm_control_5_1**: uses both baxter arms to build a 10 cup tower using cartesian coordinates **Task 2**

10.  **tag_detection**: ???  DK FILL THIS WITH YOUR NODES



### Python Package
1. **simulator.py**: uses the MoveIt to create a planning scene and perform some path planning tasks
      1. Gets the intial positions of the cups
            1. If using the real robot gets the cup positions from computer vision
            2. If using the fake robot gets the cup positions from gazebo (get_model_state service)
      2. Sets the cup and tables dimensions using scene_objects.yaml
      3. Adds a table and cups to RViz
      4. Attaches and detaches the cups 
      5. Restarts the gazebo scene
      6. Uses various functions to sort the cups into specific configurations
   
    
### Services 
<rosservice call /test_control "choice:  #id" >
There is one main service that we use in order to control the robot. This services takes as an input an id integer and based on that perform the specified task.

```   
Call test_control service with the following choices, this is a general template, you should check each arm_control version for specific instructions. See test_control_callback function.

0. print current pose of left and right arm
1. set hands at home position (grab the hands above table before calling this)
2. restart scene at state 1
3. restart scene state 2
4. left hand grab and place cup
5. right hand grab and place cup
6. both hands grab and place
7. Execute Task 2 (building tower )
8. Execute Task state 1
9. decrease step move 
10.increase step move 
11.execute step 

Debug
100. check cups_sorted
101. check assing_cup_st1
102. check create_sorting_list
103. check grab_next_pos
104. check place_next_pos
105. close both grippers
106. open both grippers
```

### Launch Files
1. baxter_world.launch
   1. opens Gazebo world file: baxter.world
   2. loads scene_objects.yaml file to obtain object parameters
   3. spawns URDF Robot into Gazebo (Baxter)
   4. publishes a static transform between the world and the base of the Baxter Robot
   5. calls a the Rethink Robotics baxter_simulator launch file used to launch a package that emulates the hardware interfaces of Baxter

   
2. build_tower.launch
   1. loads scene_objects.yaml file to obtain object parameters
   2. Activate node joint_trajectory_action_server from pkg baxter_interface
   3. Activate node gripper_action_server from pkg baxter_interface
   4. enable_robot from pkg baxter_tools
   5. demo_baxter.launch (moveIt rViz) from pkg baxter_moveit_config
3. empty_world.launch
   1. starts an empty Gazebo world
4. tagdetect.launch
   1. calls continuous_detection.lauch file from pkg apritag_ros 
   2. runs the tag_detection node from pkg tower 




# Video demonstrations
[3-Cups](https://drive.google.com/file/d/1tn9pztSq6Xdr_6aRJKh8TYW_HBiO4azu/view?usp=sharing)

[6-Cups](https://drive.google.com/file/d/1YWpt5TbLPCU3jN0AJhUhMiA8x8UWeAxr/view?usp=sharing)

[10-Cups](https://drive.google.com/file/d/1kI49MgHbEAJTUpVp9LG3RtJT45qgAd_y/view?usp=sharing)

[Cup Tower Demolition](https://drive.google.com/file/d/11vVrbHlaizXref7YBJSDJ4Ly2BNFXPnr/view?usp=sharing)





# Lessons Learned and Future Work
_________put robot faoilure videos here______________-

# External Packages
## Tags
[April_Tags_ROS](http://wiki.ros.org/apriltag_ros) package is used for 3D pose estimation of the cups in space. 
## Motion
[Moveit](https://moveit.ros.org/) package is used for motion planning of the Baxter.
## Simulation
[rviz](http://wiki.ros.org/rviz) is used as visulaization tool to check that the simulation robot workspace matches the actual robot workspace. 
[BaxterSimulator](https://sdk.rethinkrobotics.com/wiki/Baxter_Simulator) is used to locate the position of the cups in simulation and simulate Baxter motion. The group did not used the Baxter Simulator to build a cup tower becasue this goal was achieve with the actual Baxter robot. 


# References
[final project guidelines](https://nu-msr.github.io/me495_site/final_project2020.html)

[rethink workspace](https://nu-msr.github.io/me495_site/lecture13_rethink.html)

[rethinkrobotics](https://sdk.rethinkrobotics.com/wiki/Home)

[rethinkrobotics git](https://github.com/RethinkRobotics/)

[moveIt](https://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)
[planning](https://github.com/ros-planning/moveit_robots/tree/kinetic-devel/baxter/baxter_moveit_config/config)

# Team
Dimitrios Chamzas 

Dong Ho Kang

Yuxiao Lai

Gabrielle Wink




