#!/bin/sh
export TURTLEBOT_GAZEBO_MAP_FILE=/home/workspace/catkin_ws/src/World/myMap.yaml
export TURTLEBOT_3D_SENSOR=kinect
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/my_world_4" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" 
 
