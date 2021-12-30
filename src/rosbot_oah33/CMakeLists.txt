cmake_minimum_required(VERSION 3.0.2)
project(rosbot_oah33)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rosbot_data
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# executable for braitenberg_exploration cpp file (using proxy sensors)
add_executable(node_braitenberg src/node_braitenberg.cpp)

target_link_libraries(node_braitenberg
    ${catkin_LIBRARIES}
)

# 
add_executable(node_comp_results src/node_comp_results.cpp)

target_link_libraries(node_comp_results
    ${catkin_LIBRARIES}
)


#############
## Install ##
#############
# launch files folder
install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


#############
## Testing ##
#############
## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rosbot_data.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)