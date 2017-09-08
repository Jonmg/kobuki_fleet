/*
 * StateInit.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 *      Author: Jon Martin
 */

#include "state_init.h"
#include "state_idle.h"
#include "state_error.h"

#include <string>

#include <geometry_msgs/Twist.h>
#include "model/model.h"

/**
 * @file state_init.cpp
 * @brief contains implementation of StateInit
 */


StateInit::StateInit(Model* const model):
    StateFleetBase(model),
    covarianceReceived_(false),
    covariance_(0.0),
    threshCovariance_(0.0),
    initAngularVel_(0.0)
{
  std::string topicVelocity;
  std::string topicCovariance;
//  std::string getAllInventoryTopic;

  prvNh_.param<std::string>("topic_velocity",    topicVelocity,     "mobile_base/commands/velocity");
  prvNh_.param<std::string>("topic_covariance",  topicCovariance,   "probPose");
  prvNh_.param<std::string>("get_all_inventory_data_topic", getAllInventoryTopic_, "/getAllInventoryData");
  prvNh_.param<double>     ("thresh_covariance", threshCovariance_, 0.8);
  prvNh_.param<double>     ("init_angular_vel",  initAngularVel_,   0.5);

  subsCovariance_ = nh_->subscribe(topicCovariance, 1, &StateInit::callBackCovariance, this);
  pubCommandVel_ = nh_->advertise<geometry_msgs::Twist>(topicVelocity, 1);

  _getAllInventoryData = nh_->serviceClient<kobuki_fleet_msgs::getAllInventoryData>(getAllInventoryTopic_);
    if(!_getAllInventoryData.waitForExistence(ros::Duration(5)))
      ROS_ERROR_STREAM("the service ’" << getAllInventoryTopic_ << "’ is not available");

}

StateInit::~StateInit()
{

}

void StateInit::onEntry()
{
  kobuki_fleet_msgs::getAllInventoryData srv;
  srv.request.start.data = true;
  if(!_getAllInventoryData.waitForExistence(ros::Duration(5)))
      ROS_ERROR_STREAM("StateInit: the service ’" << getAllInventoryTopic_ << "’ is not available");

  while(!_getAllInventoryData.call(srv))
  {
    ROS_ERROR("StateInit: Inventory read data failed. Repeat the call");
    usleep(100000);
  }

  allLocalizations_.clear();
  for (unsigned int i=0; i<srv.response.locations.size();i++)
  {
    allLocalizations_.push_back(srv.response.locations.at(i));
  }
  ROS_INFO("StateInit: Inventory data received correctly");

}
void StateInit::onActive()
{
  ROS_INFO_ONCE(__PRETTY_FUNCTION__);
  this->publishStateStat();
  bool sim = model_->checkSimulation();
  
  if(sim == true)
  {
//    ROS_WARN_STREAM( "This is run in a simulation environment");
    ROS_INFO_STREAM("State Init. transition to stateIdle");
    _agent->transitionToVolatileState(new StateIdle(model_));
    return;
  }

  double commandAngular = 0.0;
  bool switchState = false;
  if(!covarianceReceived_)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " no answer from localization...stop motors");
    commandAngular = 0.0;
  }
  else
  {
    if(covariance_ < threshCovariance_)
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " localization not set. Covariance smaller thresh (" << covariance_
          << " < " << threshCovariance_ << ")");
      commandAngular = initAngularVel_;
    }
    else
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " localization sucessful...switch to state idle");
      commandAngular = 0.0;
      switchState = true;
    }
  }
  geometry_msgs::Twist velCmd;
  velCmd.angular.z = commandAngular;
  pubCommandVel_.publish(velCmd);
  if(switchState)
  {
    ROS_INFO_STREAM("State Init. transition to stateIdle");
    _agent->transitionToVolatileState(new StateIdle(model_));
  }
  covarianceReceived_ = false;   //reset flag. Will be set by subscriber if a message has been received
}

void StateInit::callBackCovariance(const std_msgs::Float32& msg)
{
  covarianceReceived_ = true;
  covariance_ = msg.data;
}

const kobuki_fleet_msgs::StateMachineStat StateInit::state(void)const
{
  kobuki_fleet_msgs::StateMachineStat stat;
  stat.current_state = kobuki_fleet_msgs::StateMachineStat::INIT;
  return stat;
}

