/*
 * start_RvizGUI.cpp
 *
 *  Created on: 26.05.2017
 *      Author: basti
 */

#include "RvizGUI.h"
#include <ros/ros.h>

int main (int argc, char** argv){
  ros::init(argc, argv, "rviz_gui_node");
  RvizGUI rvizGui;
  rvizGui.run();
}
