/*
 * model.cpp
 *
 *  Created on: Aug 2, 2016
 *      Author: phil
 */

#include "model.h"

/**
 * @file model.cpp
 * @brief contains implementation of class Model
 */

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

Model::Model(const std_msgs::UInt16& robotId, ros::NodeHandle& nh):
      controllerTasks_(new ControllerTasks(nh)),
      controllerHeartBeatList_(new ControllerHeartbeatList(nh)),
      robotId_(robotId),
      curTask_(NULL),
      moveBaseClient_(NULL),
      simulation_(0)
{

}

Model::~Model()
{
  delete controllerTasks_;
  delete controllerHeartBeatList_;
  delete curTask_;
  delete moveBaseClient_;
}

bool Model::setUpMoveBase(void)
{
  ros::NodeHandle prvNh("~");
  std::string topicMoveBaseAction;
  prvNh.param<std::string>("topic_move_base_action", topicMoveBaseAction, "robot0/move_base");

  moveBaseClient_ = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(topicMoveBaseAction, true);

  unsigned int ctr = 0;
  while(!moveBaseClient_->waitForServer(ros::Duration(TIME_OUT_MV_BS)))
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Waiting for the move_base action server " << topicMoveBaseAction << " to come up");
    if(ctr++ >= TRIES_REACH_MV_BS)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! cant reach move base");
      return false;
   //   throw std::string("Time out reaching move base");
      //return;
    }
  }
  return true;
}

} /* namespace bobbyrob */
