/*
 * state_machine.h
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

#ifndef ROS_SRC_KOB_SM_SRC_STATE_MACHINE_H_
#define ROS_SRC_KOB_SM_SRC_STATE_MACHINE_H_

/**
 * @file state_machine.h
 * @brief declaration of class StateMachine
 */

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include "obcore/statemachine/Agent.h"
#include "model/model.h"
#include "kobuki_fleet_msgs/ConnectionState.h"

//#include <actionlib/client/simple_action_client.h>
//#include <stdr_msgs/DeleteRobotAction.h>

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{
/**
 * @class StateMachine
 * @brief Main management class for the state machine.
 * Main managagement class for the state machine. Contains the ROS spinner and main error subscribers
 */
class StateMachine
{
public: ///todo author in class description
  /**
   * Standard constructor
   */
  StateMachine();
  /**
   * Destructor
   */
  virtual ~StateMachine();
  /**
   * @brief Method to start the node
   */
  void start(void){this->run();}

  /**
   @brief Deletes a robot by frame_id
   @param name [const std::string&] The robot frame_id to be deleted
   @return bool : True if deletion was successful
   **/
   bool deleteRobot(const std::string& name);

private:
  /**
   * @brief Contains main loop
   */
  void run(void);
  /**
   * @brief ROS callback method for the connection state
   * @param msg kobuki_fleet_msgs::ConnectionState contains connection state
   * If this callback receives a broken connection state, the state machine switches to error
   */
  void callBackConnectionState(const kobuki_fleet_msgs::ConnectionState& msg);
  /**
   * @brief Method to stop state machine
   */
  void stop(void);
  /**
   * Callback method for kill service.
   * @param req std_srvs::Empty object
   * @param res std_srvs::Empty object
   * @return true in case of success
   */
  bool callBackKillService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  ros::NodeHandle nh_;                    ///< ros::NodeHandle main node handle
  obvious::Agent* agent_;                 ///< obvious::Agent pointer to agent instance
  Model* model_;                          ///< Model pointer to shared model
  ros::Rate r_;                           ///< ros::Rate loop rate
  ros::Subscriber subsConnectionState_;   ///< ros::Subscriber ROS subscriber object
  ros::ServiceServer serverKill_;       ///< ros::ServiceServer ROS service service server object
  ros::ServiceClient unloadRobotStdrClient_;
  //actionlib::SimpleActionClient<stdr_msgs::DeleteRobotAction> deleteRobotClient_; ///< Action client for deleting robots
  int rId_;
  std::string robot_ns_;

};

}

#endif /* ROS_SRC_KOB_SM_SRC_STATE_MACHINE_H_ */
