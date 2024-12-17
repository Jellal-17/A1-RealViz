cmake_minimum_required(VERSION 3.0.2)
project(unitree_joint_publisher)

add_compile_options(-std=c++11)

# Find the Python interpreter from the virtual environment
set(PYTHON_EXECUTABLE "/home/unitree/parkour/parkour_v/bin/python")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  joint_state_publisher
  xacro
  unitree_legged_msgs
)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()
set(LEGGED_SDK_NAME -pthread libunitree_legged_sdk_${ARCH}.so lcm)

set(EXTRA_LIBS ${LEGGED_SDK_NAME})

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/unitree_legged_sdk/include
)

link_directories(${CMAKE_SOURCE_DIR}/unitree_legged_sdk/lib)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs joint_state_publisher xacro unitree_legged_msgs
)

add_executable(joint_publisher src/joint_publisher.cpp)
target_link_libraries(joint_publisher ${EXTRA_LIBS} ${catkin_LIBRARIES})

add_executable(joint_command_publisher src/joint_command_publisher.cpp)
target_link_libraries(joint_command_publisher ${EXTRA_LIBS} ${catkin_LIBRARIES})

## Install C++ executable
install(TARGETS joint_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS joint_command_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install Python scripts
catkin_install_python(PROGRAMS
  src/real_time_plot.py
  src/joint_command_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz)