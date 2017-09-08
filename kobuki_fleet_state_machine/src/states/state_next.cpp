/*
 * state_next.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

#include "state_next.h"
#include "state_drive.h"
#include "state_idle.h"
#include "state_error.h"


StateNext::StateNext(Model* model):
            StateFleetBase(model),
            subTaskActualNumber_(0)
{
	ros::NodeHandle prvNh("~");
	std::string topicsetTaskStatusService;
    prvNh.param<std::string>("topic_set_task_status_server", topicsetTaskStatusService, "/task_status+robot_id_");
    settaskStatusClient_ = nh_->serviceClient<kobuki_fleet_msgs::TaskStatus>(topicsetTaskStatusService);
    if (!settaskStatusClient_.waitForExistence(ros::Duration(2)))
    	ROS_WARN_STREAM("No possible to reach " << topicsetTaskStatusService << " service");
    else
    	ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "service " << topicsetTaskStatusService << " available.");
}

StateNext::~StateNext()
{


}

void StateNext::onActive(void)
{
  ROS_INFO_THROTTLE(5, "SM(next)");

  model_->getSubTasks(subTaskVector_);

  if (!model_->getSubTasks(subTaskVector_) || !nextTask())
  {
    //clear Subtask
    subTaskVector_.subtasks.clear();
    model_->addSubTasks(subTaskVector_);
    subTaskActualNumber_=0;

    //clear Task in control_task_node
    kobuki_fleet_msgs::TaskStatus  srvSetTaskStatus;
    srvSetTaskStatus.request.tid = model_->curTask().tid;
    srvSetTaskStatus.request.taskStatus = kobuki_fleet_msgs::Task::FINISHED;
    if (!settaskStatusClient_.waitForExistence(ros::Duration(2)))
      ROS_WARN_STREAM("StateNext. No possible to reach setStatus service");

    settaskStatusClient_.call(srvSetTaskStatus);
    kobuki_fleet_msgs::Task emptyTask;
    model_->setCurentTask(emptyTask);

    //transition to StateIdle
    ROS_INFO("SM(next): No more tasks to be done. Go to State Idle");
    _agent->transitionToVolatileState(new StateIdle(model_));
    return;
  }

  ROS_INFO("-------------------------------------------------------------------------");
  ROS_INFO_STREAM("SM(next): NewSubTask:"
              << " SubTasktType: " << actualSubTask_.subTasktType.data()
              << " ServiceArea: " << actualSubTask_.serviceArea.data()
          );
  ROS_INFO("-------------------------------------------------------------------------");

  if (actualSubTask_.subTasktType == "M")
  {
    ROS_INFO_STREAM("SM(next): task type = " << actualSubTask_.subTasktType << ". Go to State Drive");
    _agent->transitionToVolatileState(new StateDrive(model_));
  }
  else if (actualSubTask_.subTasktType == "W")
  {
    ROS_INFO_STREAM("SM(next): task type = " << actualSubTask_.subTasktType << ". Go to State Wait");
    sleep(actualSubTask_.waitingTime.sec);
//    _agent->transitionToVolatileState(new StateWait(model_));
  }
  else
  {
    ROS_INFO_STREAM("SM(next): task type = " << actualSubTask_.subTasktType << ". Go to State Error");
    _agent->transitionToVolatileState(new StateError(model_, StateError::NEXT_ERROR));
  }
}

bool StateNext::nextTask()
{
  if (!subTaskVector_.subtasks.size())
    return false;
  else
  {
    actualSubTask_ = subTaskVector_.subtasks.front();//_subTaskActualNumber;
    model_->setActualSubTask(actualSubTask_);

    ROS_INFO_STREAM(subTaskVector_.subtasks.size() << " Tasks left");
    subTaskVector_.subtasks.erase(subTaskVector_.subtasks.begin());
    model_->addSubTasks(subTaskVector_);

    return true;
  }
}

const kobuki_fleet_msgs::StateMachineStat StateNext::state(void)const
{
  kobuki_fleet_msgs::StateMachineStat stat;
  stat.current_state = kobuki_fleet_msgs::StateMachineStat::NEXT;
  return stat;
}


