/*
 * StateInit.cpp
 *
 *  Created on: Aug 1, 2016
 *      Author: phil
 */

#include "state_init.h"
#include "state_idle.h"
#include "state_error.h"

#include <string>

#include <geometry_msgs/Twist.h>

/**
 * @file state_init.cpp
 * @brief contains implementation of StateInit
 */

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

StateInit::StateInit(Model& model, ros::NodeHandle& nh):
    StateFleetBase(model, nh),
    nh_(nh),
    covarianceReceived_(false),
    covariance_(0.0),
    threshCovariance_(0.0),
    initAngularVel_(0.0)
{
  std::string topicVelocity;
  std::string topicCovariance;

  prvNh_.param<std::string>("topic_velocity",    topicVelocity,     "mobile_base/commands/velocity");
  prvNh_.param<std::string>("topic_covariance",  topicCovariance,   "probPose");
  prvNh_.param<double>     ("thresh_covariance", threshCovariance_, 0.8);
  prvNh_.param<double>     ("init_angular_vel",  initAngularVel_,   0.5);

  subsCovariance_ = nh_.subscribe(topicCovariance, 1, &StateInit::callBackCovariance, this);
  pubCommandVel_ = nh_.advertise<geometry_msgs::Twist>(topicVelocity, 1);

}

StateInit::~StateInit()
{

}

void StateInit::onActive()
{
  this->publishStateStat();
  bool sim = model_.checkSimulation();
  ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "simulation: " <<  sim << std::endl);
  
  if(sim == true)
  {
  	ROS_WARN_STREAM( "This is run in a simulation environment" << std::endl);
  	if(!model_.setUpMoveBase())
        _agent->transitionToVolatileState(new StateError(model_, nh_, StateError::INIT_ERROR));
    else
    _agent->transitionToVolatileState(new StateIdle(model_, nh_));
    
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
          << " < " << threshCovariance_ << ")" << std::endl);
      commandAngular = initAngularVel_;
    }
    else
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " localization sucessful...switch to state idle" << std::endl);
      commandAngular = 0.0;
      switchState = true;
    }
  }
  geometry_msgs::Twist velCmd;
  velCmd.angular.z = commandAngular;
  pubCommandVel_.publish(velCmd);
  if(switchState)
  {
    if(!model_.setUpMoveBase())
        _agent->transitionToVolatileState(new StateError(model_, nh_, StateError::INIT_ERROR));
    else
    _agent->transitionToVolatileState(new StateIdle(model_, nh_));
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

}
