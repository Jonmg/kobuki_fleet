cmake_minimum_required(VERSION 2.8.3)
project(kobuki_fleet_inventory)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  youbot_msgs
  kobuki_fleet_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp
)

include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(kobuki_fleet_inventory_node src/youbot_inventory_node.cpp src/youbot_inventory.cpp)

add_dependencies(kobuki_fleet_inventory_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(kobuki_fleet_inventory_node 
  ${catkin_LIBRARIES} ${OpenCV_LIBS}
)
