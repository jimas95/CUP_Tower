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

# Packages
## 1. tower
The tower package is the primary package used to control the robot and create the planning scene. 
### Nodes
1. **arm_control_2_1**: stacks cups concentrically into a stack
![](https://drive.google.com/file/d/1s5hzuRjC6xggcV8JQsPUKG9TRFy-wpE5/view?usp=sharing)
2. **arm_control_2_2**: takes cups from middle of the workspace and moves them to the side of the table (cleans the robot workspace)
3. **arm_control_2_3**: builds a 3 cup tower using cartesian coordinates
4. **arm_control_3_1**: uses computer vision (april tags) to locate the initial postion of the cups, then stack the cups into a 3 cup tower 
![](https://drive.google.com/file/d/194e5V0f9VPYthNb1bZZ_MKHvIUMdgzCD/view?usp=sharing)
5. **arm_control_3_2**: uses cartesian coordinates to locate 6 cups and place them in a tower
6. **arm_control_3_3**: uses computer vision (april tags) to locate the initial postion of the cups, then stack the cups into a 6 cup tower 
7. **arm_control_4_1**: uses cartesian coordinates and both Baxter arms to place 6 cups in a tower 
8. **arm_control_4_2**: uses both baxter arms to take 6 cups from middle of the workspace and move them to the side of the table (cleans the robot workspace)
9. **arm_control_5_1**: uses both baxter arms to build a 10 cup tower using cartesian coordinates
![]()

1.  **tag_detection**: ??? 
    
### Services 
<rosservice call /test_control> TAB
```   
call test_control service with the following choices
0. print current pose of left and right arm
1. set hands at home position (grab the hands above table before calling this)
2. restart scene workstation
3. restart scene inline
4. left hand grab and place cup
5. right hand grab and place cup
6. both hands grab and place
7. building tower (state_2)
8. close both grippers
9. open both grippers
```

### Launch Files
1. baxter_world.launch
   1. opens Gazebo world file: baxter.world
   2. loads scene_objects.yaml file to obtain object parameters
   3. spawns URDF Robot into Gazebo (Baxter)
   4. publishes a static transform between the world and the base of the Baxter Robot
   5. calls a the Rethink Robotics baxter_simulator launch file used to launch a package that emulates the hardware interfaces of Baxter
   6. 
   7. rviz?????????????????????????????????
   8. 
2. build_tower.launch
   1. loads scene_objects.yaml file to obtain object parameters
   2. ??????????????
   3. ???????
   4. 
3. empty_world.launch
   1. starts an empty Gazebo world
4. tagdetect.launch
   1. calls continuous_detection.lauch file from the apritag_ros package
   2. runs the tag_detection node

## 2. apriltag
This package scan the april tags and publishes the corresponding tf locations.


# Project Architecture

## Tags
[April_Tags](http://wiki.ros.org/apriltag_ros) package is used for 3D pose estimation of the cups in space. 
## Motion
[Moveit](https://moveit.ros.org/) package is used for motion planning of the Baxter.
## Simulation
[rviz](http://wiki.ros.org/rviz) is used as visulaization tool to check that the simulation robot workspace matches the actual robot workspace. 
[BaxterSimulator](https://sdk.rethinkrobotics.com/wiki/Baxter_Simulator) is used to locate the position of the cups in simulation and simulate Baxter motion. The group did not used the Baxter Simulator to build a cup tower becasue this goal was achieve with the actual Baxter robot. 

# Lessons Learned and Future Work
_________put robot faoilure videos here______________-


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




