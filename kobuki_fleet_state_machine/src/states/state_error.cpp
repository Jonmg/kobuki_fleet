/*
 * state_error.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

/**
 * @file state_error.cpp
 * @brief contains implementation of class StateError
 */

#include "state_error.h"

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

StateError::StateError(Model& model, ros::NodeHandle& nh, const ErrorState& state):
                        StateFleetBase(model, nh),
                        state_(state)
{
  // TODO Auto-generated constructor stub

}

StateError::~StateError()
{
  // TODO Auto-generated destructor stub
}

void StateError::onActive()
{
  this->publishStateStat();
  static bool success = false;
  switch(state_)
  {
  case CONNECTION:
    ROS_ERROR_STREAM_ONCE(__PRETTY_FUNCTION__ << " Connection error");
    break;
  case GOAL_ABORTED:
    ROS_ERROR_STREAM_ONCE(__PRETTY_FUNCTION__ << " Goal aborted");
    break;
  case GOAL_PREEMPTED:
    ROS_ERROR_STREAM_ONCE(__PRETTY_FUNCTION__ << " Goal preempted");
    break;
  case INIT_ERROR:
    ROS_ERROR_STREAM_ONCE(__PRETTY_FUNCTION__ << " Init error");
    break;
  default:
    ROS_ERROR_STREAM_ONCE(__PRETTY_FUNCTION__ << " Unknown error: " << state_);
    break;
  }
  if((!success))// && (state_ != INIT_ERROR))
  {
    if(model_.curTask())
    {
      std_msgs::UInt16 tid;
      tid.data = model_.curTask()->tid;

//      std_msgs::UInt16 rid;
//      rid.data = model_.robotId().data;

      std_msgs::UInt8 newState;
      newState.data = kobuki_fleet_msgs::Task::ERROR;

      if(!(success = model_.controllerTasks().setTaskState(tid, newState)))
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Failed to set task " << tid << " to error");
    }
  }
}

const kobuki_fleet_msgs::StateMachineStat StateError::state(void)const
{
  kobuki_fleet_msgs::StateMachineStat stat;
  stat.current_state = kobuki_fleet_msgs::StateMachineStat::ERROR;
  return stat;
}

}
