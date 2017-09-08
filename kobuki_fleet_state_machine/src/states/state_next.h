/*
 * state_next.h
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

#ifndef ROS_SRC_KOB_SM_SRC_STATES_STATE_NEXT_H_
#define ROS_SRC_KOB_SM_SRC_STATES_STATE_NEXT_H_

#include <ros/ros.h>

#include "model/model.h"
#include "state_fleet_base.h"
#include "kobuki_fleet_msgs/Task.h"

#include "kobuki_fleet_msgs/SubTask.h"
#include "kobuki_fleet_msgs/SubTaskVector.h"

///@todo: rename state next with process task vector


/**
 * @class StateNext
 * @brief Class implementing (persistent) state next to handle vectors of tasks
 * This state reads a task vector and transfers the commands to the drive or
 * docking nodes. This state is persistant. It lives until the taskvector is empty
 */
class StateNext: public StateFleetBase
{
public:
  /**
   * Constructor
   * @param model Instance of class model used for data handling
   * @param nh ros::NodeHandle reference to main node handle
   * @param task kobuki_fleet_msgs::Task pointer to current task
   */
  StateNext(Model* model);
  /**
   * Destructor
   */
  virtual ~StateNext();
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

  ros::ServiceClient settaskStatusClient_;

  /**
     * saves in the model the actualSubTask and the rest on the subTaskVector
     */
    bool nextTask();

    kobuki_fleet_msgs::SubTask actualSubTask_;

    kobuki_fleet_msgs::SubTaskVector subTaskVector_;

    int subTaskActualNumber_;
};


#endif /* ROS_SRC_KOB_SM_SRC_STATES_STATE_NEXT_H_ */
