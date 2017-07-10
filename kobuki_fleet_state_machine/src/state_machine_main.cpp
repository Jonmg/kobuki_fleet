/*
 * state_machine_main.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

#include <ros/ros.h>
#include "state_machine.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kob_state_machine");
  bobbyrob::StateMachine sm;
  sm.start();
}

