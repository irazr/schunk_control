cmake_minimum_required(VERSION 2.8.3)
project(schunk_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs 
  urdf 
  diagnostic_updater 
  schunk_libm5api 
  libntcan 
  libpcan 
  cob_srvs 
  brics_actuator 
  diagnostic_msgs 
  control_msgs
  actionlib
  actionlib_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    CATKIN_DEPENDS sensor_msgs urdf diagnostic_updater schunk_libm5api libntcan libpcan cob_srvs brics_actuator diagnostic_msgs control_msgs robot_state_publisher cob_trajectory_controller gazebo_ros rostopic joint_trajectory_action diagnostic_aggregator pr2_dashboard_aggregator pr2_controller_manager pr2_mechanism_diagnostics actionlib_msgs
    DEPENDS # TODO
    INCLUDE_DIRS include include/${PROJECT_NAME}
    LIBRARIES # TODO
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
  include/${PROJECT_NAME}
)

add_executable(
  ${PROJECT_NAME} 
  src/PowerCubeCtrl.cpp
  src/PowerCubeCtrlParams.cpp 
  src/PowerCubeChain.cpp
)

target_link_libraries(
  ${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)
  
set_target_properties(
  ${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-D__LINUX__ -DUSE_ESD"
)

#############
## Install ##
#############

install(DIRECTORY calibration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY Media
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY meshes
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############


