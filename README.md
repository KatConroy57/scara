# SCARA Final Project

* **Author**:  WPI RBE 500 2019 Fall Team 8

  ​                 Katharine Conroy, Thejus Jose, Zhuoyun Zhong

## Install

Environment: Ubuntu 16.04 + ROS Kinectic

To install gazebo controllers:

`sudo apt-get install ros-kinetic-joint-state-controller`
`sudo apt-get install ros-kinetic-effort-controllers`
`sudo apt-get install ros-kinetic-position-controllers`

Build Instruction

1. Clone this repository under **catkin_ws/src/**
2. Compile under **catkin_ws**: `catkin_make`
3. Go to path: **catkin_ws/src/scara/scara_command/src/**
4. Give py files permission: `chmod +x scara_connector.py scara_FK_server.py scara_IK_server.py joints_controller.py ` .

## Viewing and Jogging the Robot

To see the robot in rviz:

`roslaunch scara_description scara_rviz.launch`

To generate the robot in gazebo:

`roslaunch scara_gazebo scara_world.launch`

Try to control different joints by:

`rostopic pub -1 /scara/joint1_position_controller/command std_msgs/Float64 "data: 1"`

`rostopic pub -1 /scara/joint2_position_controller/command std_msgs/Float64 "data: 2"`

`rostopic pub -1 /scara/joint3_position_controller/command std_msgs/Float64 "data: 0.3"`



## Running

#### Project 1

Project 1 implements three nodes including forward kinematic, inverse kinematic and connector.

The use these rodes, run them by

`roscore`

`rosrun scara_command  scara_FK_server.py`

`rosrun scara_command  scara_IK_server.py`

`rosrun scara_command  scara_connector.py`

After opening all the nodes, there will be some services.

The first two services provide inverse kinematic and forward kinematic calculation.

`rosservice call compute_ik x, y, z, phi, theta, psi` 

`rosservice call compute_fk q1, q2, d3 `

The connector builds bridges for connecting gazebo robot and kinematic nodes. The service it provides does not need any input but requires that the gazebo is working. It takes the pose/joint variables of the robot in gazebo and calls scara_IK_server/scara_FK_server to compute the result. It will give both the computed joint variables/pose result and gazebo joint variables/pose data for comparison. If the error is less than 0.01, one could confirm that the nodes are working correctly.

`rosservice call check_ik` 

`rosservice call check_fk` 

#### Project 2

Generate scara 2 in gazebo:

`roslaunch scara_gazebo scara2_world.launch`

This time only joint 3 can be controlled and the available range for joint 3 is from 0 to 0.3 meters.

To control joint 3 with joints_controller server:

`rosrun scara_command joints_controller.py`

To give a reference position to the PD controller (JOINT_NAME = joint3) (NUMBER < 0):

`rosservice call set_joint_ref joint3 NUMBER`

One should be able to see the joint 3 moves to the desired position.

#### Project 3

Ongoing...

## Node graph (temp):

![node_graph_temp](demo/node_graph_temp.png)
