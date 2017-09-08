cmake_minimum_required(VERSION 2.8.3)
project(kobuki_fleet_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  std_msgs
  tf
  actionlib_msgs
  sensor_msgs
)

SET(CMAKE_BUILD_TYPE RELEASE)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O2 -std=c++11")

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   HeartBeat.msg
   HeartBeatList.msg 
   Task.msg
   TaskList.msg
   ConnectionState.msg
   StateMachineStat.msg
   NewTask.msg
   NewTaskList.msg
   Bidding.msg
   LocationIdentifier.msg
   SubTask.msg
   SubTaskVector.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   BiddingOffer.srv 
   AssignTask.srv
   TaskStatus.srv
   getAllInventoryData.srv
   GetCurrentTask.srv
   getInventoryData.srv
   setInventoryData.srv
   pop_subTaskVector.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   DIRECTORY action
   FILES 
	 ManagerTask.action
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  
   geometry_msgs
   actionlib_msgs
   sensor_msgs
)

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kobuki_fleet_msgs
  CATKIN_DEPENDS actionlib_msgs #message_runtime geometry_msgs roscpp std_msgs 
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

