/*
x * state_machine.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

/**
 * @file state_machine.cpp
 * @brief contains implementation of class StateMachine
 */

#include "state_machine.h"
#include "states/state_init.h"
#include "states/state_error.h"
#include "kobuki_fleet_msgs/StateMachineStat.h"
#include "nodelet/NodeletUnload.h"

#include <string>

StateMachine::StateMachine():
    nh_(new ros::NodeHandle)
 //deleteRobotClient_("stdr_server/delete_robot", true)
{
  ros::NodeHandle prvNh("~");
  double rateVar = 0.0;
  bool simulation = false;
  std::string topicConnectionState;
  std::string topicKillService;

  prvNh.param<int>("robot_id", rId_, 1);
  prvNh.param<std::string>("topic_connection_state", topicConnectionState, "/connection_state");
  prvNh.param<std::string>("tf_robot_frame", robot_ns_, "/robot0");
  prvNh.param<std::string>("topic_kill_robot_service", topicKillService, "/robot0/kill");
  prvNh.param<bool>("simulation", simulation, false);

  std::string topicRobotns = ("/robot_manager/unload_nodelet");

  subsConnectionState_ = nh_->subscribe(topicConnectionState, 1, &StateMachine::callBackConnectionState, this);
  serverKill_ = nh_->advertiseService(topicKillService, &StateMachine::callBackKillService, this);
  unloadRobotStdrClient_ = nh_->serviceClient<nodelet::NodeletUnload>(topicRobotns);
  if (!unloadRobotStdrClient_.waitForExistence(ros::Duration(2)))
    ROS_WARN_STREAM("No possible to reach " << topicRobotns << " service");

  std_msgs::UInt16 id;
  id.data = static_cast<unsigned int>(rId_);
  model_= new Model(id, nh_);
  //controllerTasks_ = new ControllerTasks(*nh_, model_);
  //controllerHeartBeatList_ = new ControllerHeartbeatList(*nh_, model_);

  model_->setSimulation(simulation);
  agent_.transitionToVolatileState(new StateInit(model_));

  prvNh.param<double>("loop_rate", rateVar, 1.0);
  r_ = new ros::Rate(rateVar);
}

StateMachine::~StateMachine()
{
  delete model_;

  //delete controllerTasks_;
  //delete controllerHeartBeatList_;

}

void StateMachine::run(void)
{
  while(ros::ok())
  {
    ros::spinOnce();   ///@todo: timer callback
    agent_.awake();
    r_->sleep();
  }

  ROS_INFO("Deleting the robot from the stdr");
    nodelet::NodeletUnload srv;
    srv.request.name = robot_ns_;
    if(!unloadRobotStdrClient_.call(srv))
      ROS_ERROR_STREAM("not possible to delete " << robot_ns_);
  //this->deleteRobot(name);
}

void StateMachine::callBackConnectionState(const kobuki_fleet_msgs::ConnectionState& msg)
{
  if(msg.state == kobuki_fleet_msgs::ConnectionState::DISCONNECTED)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Network connection broken. Stop state machine " << std::endl);
    this->stop();
    agent_.transitionToVolatileState(new StateError(model_, StateError::CONNECTION));
  }
}

void StateMachine::stop(void)
{
  //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient = model_->moveBaseClient();
  //moveBaseClient->cancelAllGoals();

  ROS_INFO("Deleting the robot from the stdr");
  nodelet::NodeletUnload srv;
  srv.request.name = robot_ns_;
  if(!unloadRobotStdrClient_.call(srv))
    ROS_ERROR_STREAM("not possible to delete " << robot_ns_);

  //Kill every node started together with the state machine:
  bool resulstSystem;
  std::string killNodeSM("rosnode kill /fleet_state_machine" + std::to_string(rId_));
  resulstSystem = system(killNodeSM.c_str());
  if (!resulstSystem) ROS_WARN("Failed to call rosnode kill /fleet_state_machine");

  std::string killNodeMF("rosnode kill /monitoring_fleet_node_" + std::to_string(rId_));
  resulstSystem = system(killNodeMF.c_str());
  if (!resulstSystem) ROS_WARN("Failed to call rosnode kill /monitoring_fleet_node");

  std::string killNodeMove("rosnode kill /robot" + std::to_string(rId_) + "_move_base");
  resulstSystem = system(killNodeMove.c_str());
  if (!resulstSystem) ROS_WARN("Failed to call rosnode kill /robot");
}

bool StateMachine::callBackKillService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " kill requested...going to error state");
  this->stop();
  //agent_.transitionToVolatileState(new StateError(model_, StateError::GOAL_PREEMPTED));
  return true;
}


 /*bool StateMachine::deleteRobot(const std::string& name)
 {

   stdr_msgs::DeleteRobotGoal goal;
   goal.name = name;

   while (!deleteRobotClient_.waitForServer(ros::Duration(1)) && ros::ok()) {
     ROS_WARN("Could not find stdr_server/delete_robot action.");
   }

   deleteRobotClient_.sendGoal(goal);

   bool success = deleteRobotClient_.waitForResult(ros::Duration(10));

   if (!success) {
     ROS_INFO("Could not delete robot, connection error...");
   }

   return deleteRobotClient_.getResult()->success;

 }*/

