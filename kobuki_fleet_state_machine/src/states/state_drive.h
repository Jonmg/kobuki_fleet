/*
 * state_drive.h
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

#ifndef ROS_SRC_KOB_SM_SRC_STATES_STATE_DRIVE_H_
#define ROS_SRC_KOB_SM_SRC_STATES_STATE_DRIVE_H_

/**
 * @file state_drive.h
 * @brief Contains declaration of class StateDrive
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

#include "model/model.h"
#include "state_fleet_base.h"

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

//namespace
//{
//  const unsigned int TRIES_REACH_MV_BS = 5; ///maximum tries to reach move base server before going into error
//  const double TIME_OUT_MV_BS = 1.0;        ///maximum time span for a waiting loop for move base
//}

/**
 * @class StateDrive
 * @brief State implementing the drive state.
 * Drive state is created with a specific goal and stays active until the goal is reached.
 * It will switch to state next after receiving.
 */
class StateDrive: public StateFleetBase
{
public:
  /**
   * Constructor
   * @param model Instance of class model used for data handling
   * @param nh ros::NodeHandle reference to main node handle
   * @param goal geometry_msgs::Pose driving goal
   */
  StateDrive(Model& model, ros::NodeHandle& nh, const geometry_msgs::Pose& goal);
  /**
   * Destructor
   */
  virtual ~StateDrive();
  /**
   * @brief called by state machine in every iteration
   * Method called by agent class in every iteration. Evaluates the received data and
   * switches to next state if necessary.
   */
  virtual void onActive();
  /**
   * @brief setup method. Sends goal to drives
   * Method which is called once by the agent. Will transfer the current goal to the
   * drives.
   */
  virtual void onEntry();
  /**
   * @brief Returns stats of current state (implementation of abstract method declared in state_fleet_base)
   * @return kobuki_fleet_msgs::StateMachineStat stats
   */
  virtual const kobuki_fleet_msgs::StateMachineStat state(void)const;
private:
  const geometry_msgs::Pose& goal_; ///< buffer for current goal
};

}

#endif /* ROS_SRC_KOB_SM_SRC_STATES_STATE_DRIVE_H_ */
