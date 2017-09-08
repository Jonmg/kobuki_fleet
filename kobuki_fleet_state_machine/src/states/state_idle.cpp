/*
 * state_idle.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

#include "state_idle.h"
#include "state_next.h"
#include "kobuki_fleet_msgs/pop_subTaskVector.h"

/**
 * @file state_idle.cpp
 * @brief contains implementation of StateIdle
 */

#include "kobuki_fleet_msgs/GetCurrentTask.h"
#include <kobuki_fleet_msgs/TaskStatus.h>

StateIdle::StateIdle(Model* const model):
StateFleetBase(model)
{
	ros::NodeHandle prvNh("~");

	std::string topicIsActiveTask;
	std::string topicGetCurrentSubTask;
	std::string topicsetTaskStatusService;

	prvNh.param<std::string>("topic_task_status", topicTaskStatus_, "/task_status_ws_");
	prvNh.param<std::string>("topic_get_current_subTask_vector_server", topicGetCurrentSubTask, "/getCurrentSubtaskTask");
  prvNh.param<std::string>("topic_set_task_status_server", topicsetTaskStatusService, "/task_status+robot_id_");

//    settaskStatusClient_ = nh_->serviceClient<kobuki_fleet_msgs::TaskStatus>(topicsetTaskStatusService);

  getCurrentSubTaskClient_ = nh_->serviceClient<kobuki_fleet_msgs::pop_subTaskVector>(topicGetCurrentSubTask);
	if (!getCurrentSubTaskClient_.waitForExistence(ros::Duration(2)))
		ROS_WARN_STREAM("No possible to reach " << topicGetCurrentSubTask << " service");
}

StateIdle::~StateIdle()
{

}

void StateIdle::onActive()
{
	this->publishStateStat();
	ROS_INFO_ONCE(__PRETTY_FUNCTION__);   //toDo: maybe integrate vector of tasks

	kobuki_fleet_msgs::pop_subTaskVector srvSubtask;
  if (!getCurrentSubTaskClient_.waitForExistence(ros::Duration(2)))
    ROS_WARN_STREAM("No possible to reach 'getCurrentSubTaskVector' service");

  if(getCurrentSubTaskClient_.call(srvSubtask))
  {
    model_->addSubTasks(srvSubtask.response.subTasks);
    model_->setCurentTask(srvSubtask.response.currentTask);
//    ROS_INFO_STREAM("StateIdle. Service call getCurrent Task and Subtask succeeded!");

    if (srvSubtask.response.status.data)
    {
      ROS_INFO_STREAM("StateIdle. Receive new SubtaskVector. switching to state next ");
      _agent->registerPersistantState(STATE_NEXT, new StateNext(model_));
      _agent->transitionToPersistantState(STATE_NEXT);
    }
    else
    {
      ROS_INFO_STREAM_THROTTLE(10, "StateIdle. Waiting for new tasks to manage");
    }
  }else
    ROS_ERROR_STREAM("StateIdle. not possible to call service getCurrentTask");

}

const kobuki_fleet_msgs::StateMachineStat StateIdle::state(void)const
{
	kobuki_fleet_msgs::StateMachineStat stat;
	stat.current_state = kobuki_fleet_msgs::StateMachineStat::IDLE;
	return stat;
}


