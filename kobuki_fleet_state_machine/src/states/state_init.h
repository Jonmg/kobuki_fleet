/*
 * StateInit.h
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

/**
 *  @file state_init.h
 *  @brief Declaration of state init
 */


#ifndef ROS_SRC_KOB_SM_SRC_STATES_STATE_INIT_H_
#define ROS_SRC_KOB_SM_SRC_STATES_STATE_INIT_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include "state_fleet_base.h"

#include "kobuki_fleet_msgs/getAllInventoryData.h"
#include "kobuki_fleet_msgs/LocationIdentifier.h"


class Model;

/**
 * @class StateInit
 * @brief Declaration of class state init
 * State init turns the robot until the localization has been successful.
 * It will call state idle after success.
 */
class StateInit: public StateFleetBase
{
public:
  /**
   * Constructor
   * @param model Instance of class model used for data handling
   * @param nh ros::NodeHandle reference to main node handle
   */
  StateInit(Model* const model);
  /**
   * Destructor
   */
  virtual ~StateInit();


  void onEntry();

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
   * @brief ROS callback method for covariance
   * @param msg not yet set @toDo: Determine correct topic and message type
   * ROS callback method for covariance message. The received covariance message is used
   * to determine, wether the initialization process has been completed successfully.
   */
  void callBackCovariance(const std_msgs::Float32& msg);
  ros::Subscriber subsCovariance_; ///< ROS subscriber object
  ros::ServiceClient _getAllInventoryData; ///< ROS Client to Inventory to obtain locations

  std::string getAllInventoryTopic_;

  bool covarianceReceived_;        ///< Flag determining receivement of covariance message
  double covariance_;              ///< current covariance
  ros::Publisher pubCommandVel_;   ///< ROS publisher object
  double threshCovariance_;        ///< threshold for the covarinance (thresh reached -> successful localization)
  double initAngularVel_;          ///< constant angular velocity used to localize the pf

  std::vector<kobuki_fleet_msgs::LocationIdentifier> allLocalizations_;
};

#endif /* ROS_SRC_KOB_SM_SRC_STATES_STATE_INIT_H_ */
