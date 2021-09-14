#!/bin/sh
killall gzserver
export TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find world)/mySmallWorld.world
export TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/map.yaml
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -e "roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
#xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &
xterm -e "roslaunch rviz_launcher view_navigation.launch " &
sleep 10
xterm  -e "rosrun add_markers_hs add_markers_hs ;bash" &
sleep 10
xterm  -e "rosrun pick_objects pick_objects;bash" &
sleep 5
#xterm  -e "rostopic echo /visualization_marker;bash" &
sleep 5
