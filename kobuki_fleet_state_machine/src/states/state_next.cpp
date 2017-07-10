/*
 * state_next.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

#include "state_next.h"
#include "state_drive.h"
#include "state_idle.h"

#include <geometry_msgs/Pose.h>

namespace bobbyrob
{

StateNext::StateNext(Model& model, ros::NodeHandle& nh):
            StateFleetBase(model, nh)
{

}

StateNext::~StateNext()
{

}

void StateNext::onActive()
{
  this->publishStateStat();
  if(!model_.curTask())
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error. Task is invalid (this should not happen)");
    return;
  }
  if(model_.curTask()->task_status == kobuki_fleet_msgs::Task::FINISHED)   //reentry)
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " curTask is finished->switch to idle");
    _agent->transitionToVolatileState(new StateIdle(model_, nh_));
    return;
  }
  ///@todo integrate task vector...currently only a single goal position is applied
  ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "switching to state drive ");
  _agent->transitionToVolatileState(new StateDrive(model_, nh_, model_.curTask()->task));
}

const kobuki_fleet_msgs::StateMachineStat StateNext::state(void)const
{
  kobuki_fleet_msgs::StateMachineStat stat;
  stat.current_state = kobuki_fleet_msgs::StateMachineStat::NEXT;
  return stat;
}

} //namespace bobbyrob
