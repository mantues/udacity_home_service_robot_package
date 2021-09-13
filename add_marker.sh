#!/bin/sh
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
xterm  -e  "export TURTLEBOT3_MODEL=waffle" &
xterm  -e  "roslaunch myrobot navigation.launch"  &
sleep 5
xterm  -e  "rosrun myrobot msgs_spawn_marker" &
sleep 5
xterm  -e  "rostopic pub -1 /spawn_marker std_msgs/Float32MultiArray '{layout:{dim:[],dim:[],dim:[]}, data:[3.16, -4.78, 5.0]}'" &
sleep 5
xterm  -e  "rostopic pub -1 /spawn_marker std_msgs/Float32MultiArray '{layout:{dim:[],dim:[],dim:[]}, data:[2.19, 1.8, 5.0]}'"
