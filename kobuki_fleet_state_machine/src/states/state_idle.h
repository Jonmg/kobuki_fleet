/*
 * state_idle.h
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

/**
 * @file state_idle.h
 * @brief contains declaration of StateIdle class
 */

#ifndef ROS_SRC_KOB_SM_SRC_STATES_STATE_IDLE_H_
#define ROS_SRC_KOB_SM_SRC_STATES_STATE_IDLE_H_

#include <ros/ros.h>
#include "model/model.h"
#include "state_fleet_base.h"
#include "kobuki_fleet_msgs/TaskList.h"
#include <kobuki_fleet_msgs/TaskStatus.h>
#include "kobuki_fleet_msgs/SubTaskVector.h"

/**
 * @class StateIdle
 * @brief Class implementing the idle state
 * StateIdle stays on until it has received a task list. It will switch to StateNext in this case
 */
class StateIdle: public StateFleetBase
{
public:
  /**
   * Constructor
   * @param model bobbyrob::Model reference to the shared data container
   * @param nh ros::NodeHandle reference to the shared main node hanlde of the state machine node
   */
  StateIdle(Model* const model);
  /**
   * Destructor
   */
  virtual ~StateIdle();
  /**
   * @brief called by state machine in every iteration
   * Method called by agent class in every iteration. Evaluates the received data and
   * switches to next state if necessary.
   */
  virtual void onActive();
  /**
   * @brief Returns stats of current state (implementation of abstract method declared in state_fleet_base)
   * @return kobuki_fleet_msgs::StateMachineStat stats
   */
  virtual const kobuki_fleet_msgs::StateMachineStat state(void)const;
private:
  /**
   * @brief ROS callback method for TaskList messages
   * @param msg kobuki_fleet_msgs::TaskList
   */
  void callBackTaskList(const kobuki_fleet_msgs::TaskList& msg);
  ros::Subscriber subsTaskList_;   /// ROS subscriber object
  ros::ServiceClient getCurrentSubTaskClient_;
  ros::ServiceClient settaskStatusClient_;

  std::string topicTaskStatus_;
};

#endif /* ROS_SRC_KOB_SM_SRC_STATES_STATE_IDLE_H_ */
