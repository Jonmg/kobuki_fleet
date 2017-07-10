/*
 * model.h
 *
 *  Created on: Aug 2, 2016
 *      Author: phil
 */


/**
 * @file model.h
 * @brief declaration of class model
 */

#ifndef SRC_STATE_MACHINE_MODEL_MODEL_H_
#define SRC_STATE_MACHINE_MODEL_MODEL_H_

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "obcore/statemachine/AgentModel.h"

#include "controller_tasks.h"
#include "controller_heartbeat_list.h"

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

namespace
{
  const unsigned int TRIES_REACH_MV_BS = 2; ///maximum tries to reach move base server before going into error
  const double TIME_OUT_MV_BS = 2.0;        ///maximum time span for a waiting loop for move base
}

/**
 * @class Model
 * @brief Data container for state machine
 * Class used as a data exchange container used by all states.
 */
class Model : public obvious::AgentModel
{
public:
  /**
   * Constructor
   * @param robotId unique id of the robot within the robot fleet
   * @param nh reference to main node handle of the state machine
   */
  Model(const std_msgs::UInt16& robotId, ros::NodeHandle& nh);
  /**
   * Destructor
   */
  virtual ~Model();
  /**
   * @brief Getter. Get reference to controller of task list
   * @return ControllerTasklist reference to the task list controller instance
   * Method used to access the task list controller class.
   */
  ControllerTasks& controllerTasks(void){return *controllerTasks_;}
  /**
   * @brief Getter. Get reference to controller of heartbeat list
   * @return ControllerHeartBeatList reference to the heart beat list controller instance
   * Method used to access the heart beat list controller
   */
  ControllerHeartbeatList& controllerHeartBeatList(void){return *controllerHeartBeatList_;}
  /**
   * @brief Getter. Get robotId
   * @return std_mgs::UInt16 reference to the robot id
   */
  const std_msgs::UInt16& robotId(void)const{return robotId_;}

  const kobuki_fleet_msgs::Task* const curTask(void)const{return curTask_;}

  void setCurentTask(const kobuki_fleet_msgs::Task& curTask){curTask_ = &curTask;}

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient(void){return moveBaseClient_;}
  
  void setSimulation(bool simulation){simulation_= simulation;}
  
  bool checkSimulation(void){return simulation_;}

  bool setUpMoveBase(void);
private:
  ControllerTasks* controllerTasks_;  ///< instance of task list controller
  ControllerHeartbeatList* controllerHeartBeatList_; ///instance of heart beat list controller
  const std_msgs::UInt16 robotId_;          ///< unique robot id
  const kobuki_fleet_msgs::Task* curTask_;
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* moveBaseClient_;  ///< ROS move base client object
  bool simulation_;
};

} /* namespace bobbyrob */

#endif /* SRC_STATE_MACHINE_MODEL_MODEL_H_ */
