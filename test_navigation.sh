#!/bin/sh
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
sleep 5
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
xterm  -e  "export TURTLEBOT3_MODEL=waffle" &
xterm  -e  "roslaunch myrobot turtlebotwithworld.launch" & 
sleep 5
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
xterm  -e  "export TURTLEBOT3_MODEL=waffle" &
xterm  -e  " roslaunch myrobot navigation.launch" 
