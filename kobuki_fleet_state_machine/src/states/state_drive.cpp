/*
 * state_drive.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

/**
 * @file state_drive.cpp
 * @brief Contains implementation of class StateDrive
 */

#include "state_drive.h"
#include "state_error.h"
#include "state_next.h"

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

StateDrive::StateDrive(Model& model, ros::NodeHandle& nh, const geometry_msgs::Pose& goal):
                                    StateFleetBase(model, nh),
                                    goal_(goal)
{

}

StateDrive::~StateDrive()
{

}

void StateDrive::onActive()
{
  this->publishStateStat();

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient = model_.moveBaseClient();
  if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std_msgs::UInt16 tid;
    tid.data = model_.curTask()->tid;
    std_msgs::UInt8 newState;
    newState.data = kobuki_fleet_msgs::Task::FINISHED;
    if(!model_.controllerTasks().setTaskState(tid, newState))
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error setting state of task " << tid << " to finished"  << std::endl);
      return;
    }

    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " goal reached. Switching to state next");
    _agent->transitionToVolatileState(new StateNext(model_, nh_));
    return;
  }
  else
  {
    if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "move base aborted goal->transition to error state");
      _agent->transitionToVolatileState(new StateError(model_, nh_, StateError::GOAL_ABORTED));
    }
    else if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::LOST)
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " goal lost");
    else if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::REJECTED)
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " goal rejected");
    else if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::ACTIVE)
      ROS_INFO_STREAM_THROTTLE(2, __PRETTY_FUNCTION__ << " goal still active");
    else if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " goal preempted");
    else if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::RECALLED)
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " goal recalled");
    else if(moveBaseClient->getState() == actionlib::SimpleClientGoalState::PENDING)
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " goal pending");
    else
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Unknown state answer from move base");
  }
}

void StateDrive::onEntry()
{
  ROS_INFO_ONCE(__PRETTY_FUNCTION__);
  this->publishStateStat();

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient = model_.moveBaseClient();
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";    ///@todo: launch file parameter for goal frame id or use task list
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose = goal_;
  moveBaseClient->sendGoal(goal);
}

const kobuki_fleet_msgs::StateMachineStat StateDrive::state(void)const
{
  kobuki_fleet_msgs::StateMachineStat stat;
  stat.current_state = kobuki_fleet_msgs::StateMachineStat::DRIVE;
  return stat;
}

}
