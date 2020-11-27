# About
A ROS project developed as part of ME495 - Embedded Systems in Robotics course at Northwestern University.

# Project Description
The goal of this project is to use the robot baxter and build a **HUGE** tower from plastic cups



# Quickstart Usage Instructions
1. Set up workspace and clone repo
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
8. Control the robot 
```
roslaunch tower baxter_world.launch
roslaunch tower build_tower.launch
rosrun tower arm_control joint_states:=robot/joint_states
```


# Packages
## 1. tower
The tower package is the primary package used to control the robot and create the planning scene. 
### Nodes
1. **arm_control_2_1**: stacks cups concentrically into a stack
![](IMB_8y2szr.GIF)
2. **arm_control_2_2**: takes cups from middle of the workspace and moves them to the side of the table (cleans the robot workspace)
3. **arm_control_2_3**: builds a 3 cup tower using cartesian coordinates
4. **arm_control_3_1**: uses computer vision (april tags) to locate the initial postion of the cups, then stack the cups into a 3 cup tower 
![](IMB_NpCaFE.gif)
5. **arm_control_3_2**: uses cartesian coordinates to locate 6 cups and place them in a tower
6. **arm_control_3_3**: uses computer vision (april tags) to locate the initial postion of the cups, then stack the cups into a 6 cup tower 
7. **arm_control_4_1**: ??? 
8. **arm_control_4_2**: ???
9. **arm_control_5_1**: uses both baxter arms to build a 10 cup tower using cartesian coordinates
![](IMB_NpCaFE.gif)
10. **tag_detection**: ??? 



### Service Usage
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
2. build_tower.launch
3. empty_world.launch
4. tagdetect.launch

# Project Architecture


## 2. apriltag
This package scan the april tags and publishes the corresponding tf locations.

 







# System Architecture
    ├── CMakeLists.txt







# kill gazebo
1. killall gzclient
2. killall rosmaster




# References
### ME 495 course notes
[final project guidelines](https://nu-msr.github.io/me495_site/final_project2020.html)
[rethink workspace](https://nu-msr.github.io/me495_site/lecture13_rethink.html)
### google doc link
[google doc](https://docs.google.com/document/d/1DyX0WEIv16zhfOnIXlYJH8nFUndHB3Xdr9HTS7mL4ks/edit?usp=sharing)
### Baxter 
[rethinkrobotics] (https://sdk.rethinkrobotics.com/wiki/Home)
[git] (https://github.com/RethinkRobotics/)
### moveIT
[moveIt](https://sdk.rethinkrobotics.com/wiki/MoveIt_Tutorial)
[planning](https://github.com/ros-planning/moveit_robots/tree/kinetic-devel/baxter/baxter_moveit_config/config)


# Team
Dimitrios Chamzas 

Dong Ho Kang

Yuxiao Lai

Gabrielle Wink




