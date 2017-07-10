/*
 * state_idle.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

#include "state_idle.h"
#include "state_next.h"

/**
 * @file state_idle.cpp
 * @brief contains implementation of StateIdle
 */

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

StateIdle::StateIdle(Model& model, ros::NodeHandle& nh):
                                StateFleetBase(model, nh)
{


  ros::NodeHandle prvNh("~");
  prvNh.param<std::string>("topic_task_status", topicTaskStatus_, "/task_status_ws_");

}

StateIdle::~StateIdle()
{

}

void StateIdle::onActive()
{
  this->publishStateStat();
  ROS_INFO_ONCE(__PRETTY_FUNCTION__);   //toDo: maybe integrate vector of tasks
  const kobuki_fleet_msgs::Task* curPrimaryTask = model_.controllerTasks().task(PRIMARY);
  const kobuki_fleet_msgs::Task* working = NULL;
  if(curPrimaryTask)
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " received primary task " << curPrimaryTask->tid << " for robot " << model_.robotId().data);
    if(!(curPrimaryTask->task_status == curPrimaryTask->OPEN))  ///@todo process other task states..maybe unnecessary for primary task
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Primary task already processed. Searching for secondary task");
      curPrimaryTask = NULL;  //set primary task to NULL so secondary is opened
    }
    else
    {
      working = curPrimaryTask;
    }
  }
  else
  {
    const kobuki_fleet_msgs::Task* curSecondaryTask = model_.controllerTasks().task(SECONDARY);
    if(curSecondaryTask)
    {
      //ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " received secondary task " << curSecondaryTask->tid << " for robot " << model_.robotId().data);

      //if FINISHED
      if(curSecondaryTask->task_status == kobuki_fleet_msgs::Task::FINISHED)
      {
        ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Secondary task already finished");
      }
      //if ERROR -> adopt the task
      else if(curSecondaryTask->task_status == kobuki_fleet_msgs::Task::ERROR)
      {
        ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " found failed secondary task with id " << curSecondaryTask->tid
            << ". Start processing this task");
        working = curSecondaryTask;
      }
      //task is still open, check heartbeat of robot
      else if((curSecondaryTask->task_status == kobuki_fleet_msgs::Task::OPEN) || (curSecondaryTask->task_status == kobuki_fleet_msgs::Task::WORKING))
      {
        std_msgs::UInt16 id;
        id.data = curSecondaryTask->rid1;
        const kobuki_fleet_msgs::HeartBeat* const heartBeat = model_.controllerHeartBeatList().heartBeat(id);
        if(!heartBeat)
        {
          ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! heart beat of robot " << id.data << " could not be found " );
          working = NULL;
        }
        else if(heartBeat->rob_status == kobuki_fleet_msgs::HeartBeat::DISCONNECTED)
        {
          ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " robot " << id.data << " is not responding. Will take over his task " << curSecondaryTask->tid);
          working = curSecondaryTask;
        }
        //else -> the robot is there
      }
      else
      {
        ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " secondary task has a state (not implemented yet) " << static_cast<unsigned int>(curSecondaryTask->task_status));
        working = NULL;
      }
    }
  }
  if (!working)
  {
    ROS_INFO_STREAM_THROTTLE(10, __PRETTY_FUNCTION__ << "No task to process. stay in idle");
    return;
  }

  ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " will set task " << working->tid << " to state working ");
  std_msgs::UInt8 newState;
  newState.data = kobuki_fleet_msgs::Task::WORKING;
  std_msgs::UInt16 taskId;
  taskId.data = working->tid;
  if(!model_.controllerTasks().setTaskState(taskId, newState))
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Changing status of state " << working->tid << " failed ");
    return;
  }
  ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "switching to state next ");
  model_.setCurentTask(*working);
  _agent->transitionToVolatileState(new StateNext(model_, nh_));
}

const kobuki_fleet_msgs::StateMachineStat StateIdle::state(void)const
{
  kobuki_fleet_msgs::StateMachineStat stat;
  stat.current_state = kobuki_fleet_msgs::StateMachineStat::IDLE;
  return stat;
}

}
