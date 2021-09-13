# RoboND-Home-Service-Robot

To clone and run -
-  git clone git clone https://github.com/BlueUnicorn7777/RoboND-Home-Service-Robot.git
-  cd RoboND-Home-Service-Robot/catkin_ws/src/
-  catkin_init_workspace 
-  cd ..
-  catkin_make
-  source devel/setup.bash
-  source src/scripts/home_service.sh


### ROS Packages used -
#### slam_gmapping
* https://github.com/ros-perception/slam_gmapping
* http://wiki.ros.org/slam_gmapping
* This package is used create a map of the existing gazebo world file. To test out the features run the catkin_ws/scripts/test_slam.sh script and drive the robot around the world. 
#### turtlebot
* https://github.com/turtlebot/turtlebot
* http://wiki.ros.org/turtlebot
* This package provides all the basic drivers for running and using a turtlebot.
#### turtlebot_interactions
* https://github.com/turtlebot/turtlebot_interactions
* http://wiki.ros.org/turtlebot_interactions
* Package supporting user side interactions with the turtlebot.
#### turtlebot_simulator
* https://github.com/turtlebot/turtlebot_simulator
* http://wiki.ros.org/turtlebot_simulator
* This package contains Launchers for Gazebo simulation of the turtlebot.

### Project Packages created

#### map
The generated map files are stored in this package and referred in the launch files from this location.
#### world 
The gazebo world file is stored in the package.     
#### pick_objects
This package will send simultaneous pickup and drop off goal locations to turtlebot robot using move_base server and SimpleActionClient.
ROS navigation stack creates a path for turtlebot based on Dijkstra's algorithm, while avoiding obstacles on its path.
#### add_marker
This package will create markers in rviz and show/hide/move them to simulate the virtual pick up and dropoff of object by turtlebot.
The task is to 
* Publish the marker at the pickup zone
* Pause 5 seconds
* Hide the marker
* Pause 5 seconds
* Publish the marker at the drop off zone
*
The original code can be found in the backup file   add_markers_test.cpp.
This code is later modifed to read the robot /odom poses and make a decision to show/hide/move the marker based on the robot location.
#### rviz_launcher
The modifed rviz files and launch script can be found here.

### Script Files -
#### launch.sh
Basic tutorial to create and test the script files.
#### test_slam.sh
Script to run the slam_gmapping and create a map of the world by teleoperating the robot around the world. After the map is created save it using  rosrun map_server map_saver <name_of_map>
This script will spawn turtlebot in the mySmallWorld world file , along with other launch files - gmapping_demo.launch , view_navigation.launch , keyboard_teleop.launch.
#### test_navigation.sh
This script will launch the mySmallWorld file along with turtlebot in gazebo. The map created by slam_gmapping is launched in rviz by setting the env variables -
TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find world)/mySmallWorld.world and TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/map.yaml. The amcl_demo.launch file will launch the amcl nodes and the navigation and localization can be tested by sending the 2D nav goal using rviz. 
#### pick_objects.sh
Run this script to watch turtlebot autonomously navigate from pickup location to drop off location. This script will launch the following files 
turtlebot_world.launch , amcl_demo.launch , view_navigation.launch and finally the pick_objects.launch. 
#### add_markers.sh

#### home_service.sh
Run this script as shown above in to Clone and Run section to see the complete simulation for the autonomous pickup and dropoff of the virtual marker object by turtlebot in mySmallWorld.
This script will lauch following files -
turtlebot_world.launch ,amcl_demo.launch,view_navigation.launch view_navigation.launch , add_markers and pick_objects.




