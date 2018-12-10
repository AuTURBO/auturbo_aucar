cmake_minimum_required(VERSION 2.8.3)
project(aucar_motor_driver)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  aucar_description
  geometry_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aucar_motor_driver
  CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp std_msgs tf aucar_description
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#add_executable(test src/test.cpp)
#target_link_libraries(test ${catkin_LIBRARIES})
#add_dependencies(aucar_motor_driver test.cpp)
