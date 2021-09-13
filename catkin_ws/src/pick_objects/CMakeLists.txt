cmake_minimum_required(VERSION 3.0.2)
project(pick_objects)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp move_base_msgs actionlib )

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp move_base_msgs actionlib 
  DEPENDS
)

###########
## Build ##
###########

add_executable(pick_objects src/pick_objects.cpp)
target_link_libraries(pick_objects ${catkin_LIBRARIES})


