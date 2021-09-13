#!/bin/sh
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
xterm  -e  "export TURTLEBOT3_MODEL=waffle" &
sleep 5
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
xterm  -e  "export TURTLEBOT3_MODEL=waffle" &
xterm  -e  "roslaunch myrobot turtlebotwithworld.launch" & 
sleep 5
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
xterm  -e  "export TURTLEBOT3_MODEL=waffle" &
xterm  -e  "roslaunch turtlebot3_slam turtlebot3_slam.launch" & 
sleep 5
xterm  -e  "source /opt/ros/kinetic/setup.bash" &
xterm  -e  "source ~/Desktop/turtleworld/devel/setup.bash" &
xterm  -e  "export TURTLEBOT3_MODEL=waffle" &
xterm  -e  "roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch" 
