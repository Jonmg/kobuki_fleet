/*
 * state_base.h
 *
 *  Created on: Aug 3, 2016
 *      Author: phil
 */

#ifndef SRC_STATE_MACHINE_STATES_STATE_FLEET_BASE_H_
#define SRC_STATE_MACHINE_STATES_STATE_FLEET_BASE_H_

/**
 * @file state_fleet_base.h
 * @brief Contains declaration of class StateFleetBase
 */

#include <ros/ros.h>

#include "obcore/statemachine/states/StateBase.h"

#include "model/model.h"
#include "kobuki_fleet_msgs/StateMachineStat.h"

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

/**
 * @class StateFleetBase
 * @brief Base class for state machine states
 * Base class for state machine states. Contains all shared objects such as ROS
 * Node Handle or reference to the model.
 */
class StateFleetBase: public obvious::StateBase
{
public:
  /**
   * @brief Constructor
   * @param model Model Reference to current model
   * @param nh ros::NodeHandle
   */
  StateFleetBase(Model& model, ros::NodeHandle& nh);
  /**
   * @brief Destructor
   */
  virtual ~StateFleetBase();
  /**
   * @brief Abstract method for getting specific information of the derived state
   * @return kobuki_fleet_msgs stats of the states
   */
  virtual const kobuki_fleet_msgs::StateMachineStat state(void)const = 0;
protected:
  /**
   * @brief Method to publish stats of the derived state.
   * Method to publish stats of the derived state. Called by derived state.
   */
  void publishStateStat(void);
  Model& model_;                       ///< Reference to shared model instance
  ros::NodeHandle& nh_;                /// Reference to shared node handle
  ros::NodeHandle prvNh_;              /// private node handle for access to ROS parameter server
  ros::Publisher pubStateMachineStat_; /// ROS publisher object
};

} /* namespace bobbyrob */

#endif /* SRC_STATE_MACHINE_STATES_STATE_FLEET_BASE_H_ */
