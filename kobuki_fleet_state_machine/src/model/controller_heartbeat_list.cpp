/*
 * controller_heartbeat.cpp
 *
 *  Created on: Aug 2, 2016
 *      Author: phil
 */

/**
 * @file controller_heartbeat_list.cpp
 * @brief contains implementation of class ControllerHeartbeatList
 */

#include "controller_heartbeat_list.h"

#include <string>

ControllerHeartbeatList::ControllerHeartbeatList(ros::NodeHandle& nh, Model* model):
            nh_(nh),
            model_(model),
            heartBeatList_(NULL)
{
  ros::NodeHandle prvNh("~");
  std::string topicHeartBeatList;
  prvNh.param<std::string>("topic_heart_beat_list", topicHeartBeatList, "/heart_beat_list");

  subsHeartBeatList_ = nh_.subscribe(topicHeartBeatList, 1, &ControllerHeartbeatList::callBackHeartBeatList, this);
  //pubHeartBeatList_ = nh_.advertise<kobuki_fleet_msgs::HeartBeatList>(topicHeartBeatList, 1);

}

ControllerHeartbeatList::~ControllerHeartbeatList()
{
  delete heartBeatList_;
}

const kobuki_fleet_msgs::HeartBeat* const ControllerHeartbeatList::heartBeat(const std_msgs::UInt16& robotId)const
{
  if(!heartBeatList_)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! No Heartbeatlist received yet" << std::endl);
    return NULL;
  }
  const kobuki_fleet_msgs::HeartBeat* ptr = NULL;
  for(auto iter = heartBeatList_->heartBeatList.begin(); iter < heartBeatList_->heartBeatList.end(); iter++)
  {
    if(iter->rid == robotId.data)
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " found robot with id" << robotId.data << std::endl);
      ptr = &*iter;
      break;
    }
  }
  if(!ptr)
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error. No robot found for id " << robotId.data << std::endl);
  return ptr;
}

//kobuki_fleet_msgs::HeartBeat* const ControllerHeartbeatList::heartBeat(const std_msgs::UInt16& robotId)
//{
//  if(!heartBeatList_)
//  {
//    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! No Heartbeatlist received yet" << std::endl);
//    return NULL;
//  }
//  kobuki_fleet_msgs::HeartBeat* ptr = NULL;
//  for(auto iter = heartBeatList_->heartBeatList.begin(); iter < heartBeatList_->heartBeatList.end(); iter++)
//  {
//    if(iter->rid == robotId.data)
//    {
//      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " found robot with id" << robotId.data << std::endl);
//      ptr = &*iter;
//      break;
//    }
//  }
//  if(!ptr)
//    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error. No robot found for id " << robotId.data << std::endl);
//  return ptr;
//}

//bool ControllerHeartbeatList::updateRobotStatus(const std_msgs::UInt16& robotId, const std_msgs::UInt8& newState)
//{
//  kobuki_fleet_msgs::HeartBeat* ptr = this->heartBeat(robotId);
//  if(!ptr)
//  {
//    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! No robot found for id " << robotId << std::endl);
//    return false;
//  }
//  ptr->header.stamp = ros::Time::now();
//  ptr->rob_status = newState.data;
//  return true;
//}

//bool ControllerHeartbeatList::robotWatchDog(const std_msgs::UInt16& robotId)
//{   //todo: two methods with nearly the same code...maybe a better solution.
//  kobuki_fleet_msgs::HeartBeat* ptr = this->heartBeat(robotId);
//  if(!ptr)
//  {
//    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! No robot found for id " << robotId << std::endl);
//    return false;
//  }
//  ptr->header.stamp = ros::Time::now();
//  return true;
//}

void ControllerHeartbeatList::callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList& msg)
{
  if(!heartBeatList_)
    heartBeatList_ = new kobuki_fleet_msgs::HeartBeatList(msg);
  else
    *heartBeatList_ = msg;
}
