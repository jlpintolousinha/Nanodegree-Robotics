#!/bin/sh
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/World/my_world_4" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e "rosrun rviz rviz -d /home/workspace/catkin_ws/src/RvizConfig/markers.rviz" &
sleep 5
xterm -e "rosrun pick_objects pick_objects_node" 
 
