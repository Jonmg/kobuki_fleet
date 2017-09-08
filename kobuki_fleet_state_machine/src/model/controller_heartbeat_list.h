/*
 * controller_heartbeat.h
 *
 *  Created on: Aug 2, 2016
 *      Author: phil
 */

#ifndef SRC_STATE_MACHINE_MODEL_CONTROLLER_HEARTBEAT_LIST_H_
#define SRC_STATE_MACHINE_MODEL_CONTROLLER_HEARTBEAT_LIST_H_

/**
 * @file controller_heartbeat_list.h
 * @brief contains declaration of class ControllerHeartBeatList
 */

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>

#include "kobuki_fleet_msgs/HeartBeatList.h"

#include "model.h"

/**
 * @class ControllerHeartBeat
 * @brief Class for managing heartbeat list
 * Manages the heartbeat list. Subscribes it and manages access. Will republish
 * in case the state machine updates an object in the list
 */
class ControllerHeartbeatList
{
public:
  /**
   * Constructor
   * @param nh ros::NodeHandle reference to main NodeHandle
   */
  ControllerHeartbeatList(ros::NodeHandle& nh, Model* model);
  /**
   * Destructor
   */
  virtual ~ControllerHeartbeatList();
  /**
   * @brief Getter. Return current heartbeat to specific robot ID
   * @param robotId robotId of the required heartbeat
   * @return kobuki_fleet_msgs::HeartBeat reference to the specific information or NULL in case of unknown ID
   * Returns reference to a certain heartbeat message denoted by given robot ID. Will return NULL in case
   * of an unknown ID
   */
  const kobuki_fleet_msgs::HeartBeat* const heartBeat(const std_msgs::UInt16& robotId)const;
  /**
   * @brief Method to alter a robot state. Will update the time stamp as well
   * @param robotId std_msgs::UInt8 unique robot Id
   * @param newState std_msgs::UInt8 new state
   * @return bool true in case of success, false in case of unkown robotId
   */
  bool updateRobotStatus(const std_msgs::UInt16& robotId, const std_msgs::UInt8& newState);
  /**
   * @brief Method to update the timestamp of a heartbeat
   * @param robotId std_msgs::UInt8 unique robot Id
   * @return bool true in case of success, false in case of unkown robotId
   * Malfunctioning robots can be detected by an outdated time stamp in their heartbeat.
   * This method works like a watchdog, as state machine will update the stamp in every cycle.
   * If the time stamp is outdated, there is a certain possibility, that the robot has died.
   */
  bool robotWatchDog(const std_msgs::UInt16& robotId);
private:
  /**
   * @brief ROS callback for heartbealist
   * @param msg kobuki_fleet_msgs::HeartBeatList heartbeatlist message
   * Callback for receiving the current heartbeatlist. Will initialize the buffer at
   * first call.
   */
  void callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList& msg);
//  /**
//   * @brief internal getter to access and alter heart beat list
//   * @param robotId robotId std_msgs::UInt8 unique robot Id
//   * @return bool true in case of success, false in case of unkown robotId
//   */
//  kobuki_fleet_msgs::HeartBeat* const heartBeat(const std_msgs::UInt16& robotId);
  ros::NodeHandle& nh_;                                 ///< reference to main node handle
  kobuki_fleet_msgs::HeartBeatList* heartBeatList_;  ///< buffer for storing the current heartbeatlist
  ros::Subscriber subsHeartBeatList_;                   ///< ROS subscriber object
  //ros::Publisher pubHeartBeatList_;                     ///< ROS publisher objec (used for republishing heartbeatlist)

  Model* const model_;
};


#endif /* SRC_STATE_MACHINE_MODEL_CONTROLLER_HEARTBEAT_LIST_H_ */
