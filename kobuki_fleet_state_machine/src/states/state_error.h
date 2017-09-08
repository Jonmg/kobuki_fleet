/*
 * state_error.h
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

#ifndef ROS_SRC_KOB_SM_SRC_STATES_STATE_ERROR_H_
#define ROS_SRC_KOB_SM_SRC_STATES_STATE_ERROR_H_

/**
 * @file state_error.h
 * @brief contains declaration of StateError
 */

#include <ros/ros.h>
#include "model/model.h"
#include "state_fleet_base.h"


class StateError: public StateFleetBase
{
public:
  /**
   * @enum ErrorState
   * @brief Enumeration type for different error states
   */
  enum ErrorState
  {
    CONNECTION = 0,//!< CONNECTION
    GOAL_ABORTED,   //!< GOAL_ABORTED
    GOAL_PREEMPTED,
    INIT_ERROR,
    NEXT_ERROR
  };
  /**
   * Constructor
   * @param model Instance of class model used for data handling
   * @param nh ros::NodeHandle reference to main node handle
   * @param state ErrorState reason for entering the error state
   */
  StateError(Model* const model, const ErrorState& state);
  /**
   * Destructor
   */
  virtual ~StateError();
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
  const ErrorState state_;  /// current error state
};

#endif /* ROS_SRC_KOB_SM_SRC_STATES_STATE_ERROR_H_ */
