# RoboND-Home-Service-Robot
This project demonstrates the use of several ros packages including slam_gmapping , amcl , map_server , turtlebot , turtlebot_interactions , turtlebot_simulator etc to create a simulated home service robot. The home service robot ( turtlebot for this project) will autonomously navigate it’s world and localize itself in the map file. The pick_object node publishes pickup and drop off goal locations to turtlebot robot using move_base server and SimpleActionClient. ROS navigation stack creates a path for turtlebot based on Dijkstra's algorithm, while avoiding obstacles on its path.  
The final simulation will show -  
A marker at the pickup zone.  
Will hide the marker once the robot reaches the pickup zone.  
Wait 5 seconds to simulate a pickup.  
Will show the drop off marker at the drop off zone once your robot reaches there.  


### Dependencies for Running Locally
1. Ubuntu 16.04
2. ROS Kinetic, Gazebo  
3. Cmake >3.0.2 & g++/gcc, C++11 
4. Install xterm sudo apt-get install xterm 
5. Install some dependencies  
`rosdep -i install gmapping`  
`rosdep -i install turtlebot_teleop`  
`rosdep -i install turtlebot_rviz_launchers`    
`rosdep -i install turtlebot_gazebo`    
`sudo apt-get install ros-${ROS_DISTRO}-map-server`  
`sudo apt-get install ros-${ROS_DISTRO}-amcl`  
`sudo apt-get install ros-${ROS_DISTRO}-move-base `   
`sudo apt-get install ros-${ROS_DISTRO}-slam-gmapping`  

### To clone and run
`git clone git clone https://github.com/BlueUnicorn7777/RoboND-Home-Service-Robot.git`  
`cd RoboND-Home-Service-Robot/catkin_ws/src/`  
`catkin_init_workspace`   
`cd ..`  
`catkin_make`  
`source devel/setup.bash`  
`source src/scripts/home_service.sh`  


### Official ROS Packages used -
#### slam_gmapping  
This package is used create a map of the existing gazebo world file. To test out the features run the catkin_ws/scripts/test_slam.sh script and drive the robot around the world. The script will launch a gmapping_demo.launch on turtlebot equipped with laser range finder sensors or RGB-D cameras to perform SLAM and build a map of the environment.    
https://github.com/ros-perception/slam_gmapping  
http://wiki.ros.org/slam_gmapping  
#### turtlebot  
This package provides all the basic drivers for running and using a turtlebot. We are using the turtlebot_teleop , with the keyboard_teleop.launch file provided by this package to manually drive the robot using keyboard commands.    
https://github.com/turtlebot/turtlebot  
http://wiki.ros.org/turtlebot  
#### turtlebot_interactions
This package provides all the supporting user side interactions with the turtlebot. We are using the turtlebot_rviz_launchers, with the view_navigation.launch file provided by this package. The rviz file provides a preconfigured rviz workspace including the robot model, trajectories, and map. The customized rviz_launcher package is a carry over and modified versions of the rviz config and launch files from this package.    
https://github.com/turtlebot/turtlebot_interactions  
http://wiki.ros.org/turtlebot_interactions  
#### turtlebot_simulator  
This package contains Launchers for Gazebo simulation of the turtlebot. We are the turtlebot_world.launch to deploy a turtlebot in a gazebo environment. The workld file is linked by setting the env variable  `TURTLEBOT_GAZEBO_WORLD_FILE=$(rospack find world)/mySmallWorld.world`.   
https://github.com/turtlebot/turtlebot_simulator  
http://wiki.ros.org/turtlebot_simulator  


### Project Packages created

#### map
The generated map files are stored in this package and referred in the launch files from this location.
#### world 
The gazebo world file is stored in the package and referred in the launch files from this location.     
#### pick_objects
This package will send simultaneous pickup and drop off goal locations to turtlebot robot using move_base server and SimpleActionClient.
ROS navigation stack creates a path for turtlebot based on Dijkstra's algorithm, while avoiding obstacles on its path.
#### add_marker
This package will create markers on /visualization_marker topic and show/hide/move them to simulate the virtual pick up and dropoff of object by turtlebot. The markers are shown in rviz to simulate the effect of robot picking up a virtual object and dropping it over to drop off location.
#### rviz_launcher
The modifed rviz files and launch script can be found here.  

### Localization, mapping and navigation  
AMCL  
http://wiki.ros.org/amcl

gmapping

navigation


### Script Files -
#### launch.sh
Basic tutorial to create and test the script files.
#### test_slam.sh
![mapping and localization](https://github.com/BlueUnicorn7777/RoboND-Home-Service-Robot/blob/main/gif/mapping.gif)  
This script is used to run the slam_gmapping and create a map of the world by teleoperating the robot around the world. After the map is created save it using  `rosrun map_server map_saver <name_of_map>`
This script will spawn turtlebot in the mySmallWorld world file , along with other launch files - gmapping_demo.launch , view_navigation.launch , keyboard_teleop.launch.

#### test_navigation.sh
![Navigation](https://github.com/BlueUnicorn7777/RoboND-Home-Service-Robot/blob/main/gif/navigation.gif)    

This script will launch the mySmallWorld file along with turtlebot in gazebo. The map created by slam_gmapping is launched in rviz by setting the env variables -
`TURTLEBOT_GAZEBO_MAP_FILE=$(rospack find map)/map.yaml`. The amcl_demo.launch file will launch the amcl nodes and the navigation and localization can be tested by sending the 2D nav goal using rviz. 
#### pick_objects.sh
![pick_objects](https://github.com/BlueUnicorn7777/RoboND-Home-Service-Robot/blob/main/gif/pick_objects.gif)  
Run this script to watch turtlebot autonomously navigate from pickup location to drop off location. This script will launch the following files 
turtlebot_world.launch , amcl_demo.launch , view_navigation.launch and finally the pick_objects.launch. 

#### add_markers.sh
![add_markers](https://github.com/BlueUnicorn7777/RoboND-Home-Service-Robot/blob/main/gif/add_markers.gif)  
Run this script and onserve following-   
Publish the marker at the pickup zone  
Pause 5 seconds  
Hide the marker  
Pause 5 seconds  
Publish the marker at the drop off zone 
#### home_service.sh
![home_service](https://github.com/BlueUnicorn7777/RoboND-Home-Service-Robot/blob/main/gif/home_service.gif)  

Run this script as shown above in to Clone and Run section to see the complete simulation for the autonomous pickup and dropoff of the virtual marker object by turtlebot in mySmallWorld. This script will lauch following files - turtlebot_world.launch ,amcl_demo.launch,view_navigation.launch view_navigation.launch , add_markers and pick_objects.  
The task is to -  
Initially show the marker at the pickup zone.  
Hide the marker once your robot reach the pickup zone.  
Wait 5 seconds to simulate a pickup.  
Show the marker at the drop off zone once your robot reaches it.  



