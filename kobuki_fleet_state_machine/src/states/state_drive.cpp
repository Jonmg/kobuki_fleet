/*
 * state_drive.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

/**
 * @file state_drive.cpp
 * @brief Contains implementation of class StateDrive
 */

#include "state_drive.h"
#include "state_error.h"
#include "state_next.h"


StateDrive::StateDrive(Model* const model):
   StateFleetBase(model)
{
  ros::NodeHandle prvNh("~");
  std::string topicMoveBaseAction;
  prvNh.param<std::string>("topic_move_base_action", topicMoveBaseAction, "robot0/move_base");

  moveBaseClient_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(topicMoveBaseAction, true);
  if (!moveBaseClient_->waitForServer(ros::Duration(2)))
      ROS_WARN_STREAM("No possible to reach " << topicMoveBaseAction << " service");
}

StateDrive::~StateDrive()
{
  delete moveBaseClient_;
}

void StateDrive::onEntry()
{
  ROS_INFO_ONCE(__PRETTY_FUNCTION__);
  this->publishStateStat();

  //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient = model_->moveBaseClient();
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";    ///@todo: launch file parameter for goal frame id or use task list
  goal.target_pose.header.stamp = ros::Time::now();
  kobuki_fleet_msgs::SubTask subtask;
  model_->getActualSubTask(subtask);
  goal.target_pose.pose = subtask.poseNew;
  moveBaseClient_->sendGoal(goal);
}

void StateDrive::onActive()
{
  ROS_INFO_ONCE(__PRETTY_FUNCTION__);
  this->publishStateStat();

  //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient = model_->moveBaseClient();
  if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {

    kobuki_fleet_msgs::Task curTask= model_->curTask();
    curTask.taskStatus = kobuki_fleet_msgs::Task::FINISHED;
    if(!model_->setCurentTask(curTask))
    {
      ROS_ERROR_STREAM("stateDrive: error setting state of task " << curTask.tid << " to finished");
      return;
    }

    ROS_INFO_STREAM("stateDrive: goal reached. Switching to state next");
    _agent->transitionToPersistantState(STATE_NEXT);
    return;
  }
  else
  {
    if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::ABORTED)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "move base aborted goal->transition to error state");
      _agent->transitionToVolatileState(new StateError(model_, StateError::GOAL_ABORTED));
    }
    else if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::LOST)
      ROS_INFO_STREAM("stateDrive: goal lost");
    else if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::REJECTED)
      ROS_INFO_STREAM("stateDrive: goal rejected");
    else if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE)
      ROS_INFO_STREAM_THROTTLE(5, "stateDrive: goal still active");
    else if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::PREEMPTED)
      ROS_INFO_STREAM("stateDrive: goal preempted");
    else if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::RECALLED)
      ROS_INFO_STREAM("stateDrive: goal recalled");
    else if(moveBaseClient_->getState() == actionlib::SimpleClientGoalState::PENDING)
      ROS_INFO_STREAM("stateDrive: goal pending");
    else
      ROS_ERROR_STREAM("stateDrive: Unknown state answer from move base");
  }
}

const kobuki_fleet_msgs::StateMachineStat StateDrive::state(void)const
{
  kobuki_fleet_msgs::StateMachineStat stat;
  stat.current_state = kobuki_fleet_msgs::StateMachineStat::DRIVE;
  return stat;
}

