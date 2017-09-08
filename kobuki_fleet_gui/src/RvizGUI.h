/*
 * RvizGUI.h
 *
 *  Created on: 26.05.2017
 *      Author: basti
 */

#ifndef KOBUKI_FLEET_GUI_SRC_RVIZGUI_H_
#define KOBUKI_FLEET_GUI_SRC_RVIZGUI_H_

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include "ros/ros.h"
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <iterator>
#include <string>
#include <vector>
#include "kobuki_fleet_msgs/TaskList.h"
#include "kobuki_fleet_msgs/HeartBeat.h"
#include "kobuki_fleet_msgs/HeartBeatList.h"
#include <resource_retriever/retriever.h>

class RvizGUI
{
public:
  RvizGUI();
  virtual ~RvizGUI();

  void run();

  /**
   * @brief ROS callback for task list message
   * @param msg ohm_kobuki_fleet::TaskList tasks
   * ROS Callback method to receive task lists
   */
  void callBackTaskList(const kobuki_fleet_msgs::TaskListConstPtr& msg);

  /**
   * @brief ROS callback for heartbeat list message
   * @param msg kobuki_fleet_msgs::HeartBeatList
   * ROS callback method to receive heartbeat lists
   */
  void callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList::ConstPtr& msg);

  /**
   * @brief calback function for heartbeat message from all robots
   * @param message container
   */
  void callBackHeartBeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg);

  /**
   * @brief calback function for machine heartbeat message from all machines
   * @param message container
   */
  void callBackMachineBackHeartBeat(const kobuki_fleet_msgs::TaskConstPtr& msg);


  bool printInformation(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

private:
  ros::Duration removeMarkerDuration_;
  ros::Duration disconectionDuration_;

  ros::NodeHandle n_;

  ros::Subscriber sub1_;
  //ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Publisher pubMarkerArray_;


  visualization_msgs::MarkerArray tasks_marker_array;
  visualization_msgs::MarkerArray robots_marker_array;

  kobuki_fleet_msgs::TaskList g_tasklist;  //global
  kobuki_fleet_msgs::TaskList h_tasklist; //history of tasks
  kobuki_fleet_msgs::HeartBeatList g_heartbeatlist;  //global
  kobuki_fleet_msgs::HeartBeatList h_heartbeatlist;  //history of heartbeats
};

#endif /* KOBUKI_FLEET_GUI_SRC_RVIZGUI_H_ */
