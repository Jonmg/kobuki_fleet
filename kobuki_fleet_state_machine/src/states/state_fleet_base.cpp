/*
 * state_base.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: phil
 */

/**
 * @file state_fleet_base.cpp
 * @brief contains implementation of class StateFleetBase
 */

#include "state_fleet_base.h"

#include "kobuki_fleet_msgs/StateMachineStat.h"

#include <string>

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

StateFleetBase::StateFleetBase(Model& model, ros::NodeHandle& nh):
    model_(model),
    nh_(nh),
    prvNh_("~")
{
  std::string topicStateMachineStat;
  prvNh_.param<std::string>("topic_state_machine_stat", topicStateMachineStat, "state_machine_stat");
  pubStateMachineStat_ = nh_.advertise<kobuki_fleet_msgs::StateMachineStat>(topicStateMachineStat, 1);
}

StateFleetBase::~StateFleetBase()
{

}

void StateFleetBase::publishStateStat(void)
{
  kobuki_fleet_msgs::StateMachineStat stat = this->state();
  pubStateMachineStat_.publish(stat);
}

} /* namespace bobbyrob */
