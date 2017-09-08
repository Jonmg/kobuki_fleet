/*
 * controller_tasks_node.cpp
 *
 *  Created on: Aug 30, 2017
 *      Author: Jon
 */


#include <ros/ros.h>
#include "controller_tasks.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_tasks_node");
  ControllerTasks controllerTasks;
  controllerTasks.run();
}
