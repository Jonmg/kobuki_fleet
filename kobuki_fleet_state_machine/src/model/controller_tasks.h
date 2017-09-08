/*
 * controller_tasks.h
 *
 *  Created on: Mar 2, 2017
 *      Author: jon
 */

#ifndef CONTROLLERTASKS_H_
#define CONTROLLERTASKS_H_

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include "kobuki_fleet_msgs/Task.h"
#include "kobuki_fleet_msgs/TaskList.h"
#include "kobuki_fleet_msgs/NewTask.h"
#include "kobuki_fleet_msgs/NewTaskList.h"
#include "kobuki_fleet_msgs/BiddingOffer.h"
#include <kobuki_fleet_msgs/AssignTask.h>
#include <kobuki_fleet_msgs/TaskStatus.h>
#include "kobuki_fleet_msgs/HeartBeatList.h"
#include "kobuki_fleet_msgs/GetCurrentTask.h"
#include "kobuki_fleet_msgs/getAllInventoryData.h"
#include "kobuki_fleet_msgs/LocationIdentifier.h"
#include "kobuki_fleet_msgs/SubTask.h"
#include "kobuki_fleet_msgs/SubTaskVector.h"
#include "kobuki_fleet_msgs/pop_subTaskVector.h"

#include <nav_msgs/GetPlan.h>
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "model.h"

//#include "task_action_server.cpp"

///**
// * @enum TaskOrder
// * @brief Enumeration type for order of tasks
// */
//enum TaskOrder
//{
//  PRIMARY = 1,//!< PRIMARY
//  SECONDARY = 2  //!< SECONDARY
//};

/**
 * @class task controller
 * @brief Class for managing the tasks
 */
class ControllerTasks
{
public:
  /**
   * Constructor
   * @param nh ros::NodeHandle reference to the shared node handle of the state machine
   */
//  ControllerTasks(ros::NodeHandle& nh, Model* model);
  ControllerTasks();
  /*
   * Destructor
   */
  virtual ~ControllerTasks();

  void run(void);

  /**
   * Getter. Get task from the task list for a specific robot
   * @brief Get robot specific task (use robot ID)
   * @param id unsigned int
   * @return ohm_kobuki_fleet::Task task or NULL in case no task with the id exists
   */
  kobuki_fleet_msgs::Task* const task(const TaskOrder& order);

  /**
   * Set state of a specific task to a new status.
   * @brief Set state of a specific task
   * @param tid std_msgs::UInt16 task id
   * @param rid std_msgs::UInt16 robot id
   * @param newState new state status
   * @return true in case of success, false in case rid and tid do not match or tid does not exist
   */
  bool setTaskState(const std_msgs::UInt16& tid, std_msgs::UInt8& newState);

  /**
   * @return the status of a specific task
   * @param tid std_msgs::UInt16 task id
   */
  std_msgs::UInt8  getTaskState(const std_msgs::UInt16 tid);


private:
  /**
   * ROS callBack method for a new task list. Stores the received task in a member buffer
   * @brief ROS callback method for new tasks
   * @param msg kobuki_fleet_msgs::NewTask
   */
  void callBackNewTask(const kobuki_fleet_msgs::NewTask& msg);

  /**
     * @brief ROS callback for heartbealist
     * @param msg kobuki_fleet_msgs::HeartBeatList heartbeatlist message
     * Callback for receiving the current heartbeatlist. Will initialize the buffer at
     * first call.
     */
    void callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList& msg);

  bool receiveAssignationServer(kobuki_fleet_msgs::AssignTask::Request  &req,
      kobuki_fleet_msgs::AssignTask::Response &res);

  bool getCurrentSubTaskServer(kobuki_fleet_msgs::pop_subTaskVector::Request  &req,
		  kobuki_fleet_msgs::pop_subTaskVector::Response &res);

  bool setTaskStatusServer(kobuki_fleet_msgs::TaskStatus::Request  &req,
      kobuki_fleet_msgs::TaskStatus::Response &res);

  int findCostsSrc2M(std::string machineFileName, int materialType, kobuki_fleet_msgs::Task *task);

  int findPath(geometry_msgs::Pose initPose, geometry_msgs::Pose finalPose);

  void createBNTsubtasks (kobuki_fleet_msgs::Task task, kobuki_fleet_msgs::SubTaskVector *subTasks);

  ros::NodeHandle nh_;//& nh_                  ///< reference to the main node handle
  ros::Subscriber subsNewTask_;          ///< ROS subscriber to a newTask
  ros::ServiceClient planSrvClient_;
  ros::ServiceClient biddingClient_;     //<bidd for a new task
  ros::ServiceServer taskStatusServer_;
  ros::ServiceClient taskStatusClient_;  //<inform the machine from a change on a task
  ros::ServiceServer assignmentServer_;  //<receive the assignment for a new task
  ros::Subscriber subsHeartBeatList_;    ///< ROS subscribe to Heart beat list
  ros::ServiceServer getCurrentSubTaskServer_;
  ros::ServiceClient _getAllInventoryData; ///< ROS Client to Inventory to obtain locations

  tf::TransformListener _listener;
  std::string _tfBaseFrame;
  std::string _tfRobotFrame;
  std::string robotName_;
  std::string biddOfferTopic_;
  std::string assignTopic_;
  std::string taskStatusClientTopic_;
  std::string getAllInventoryTopic_;
  int robot_id_;

  kobuki_fleet_msgs::Task primary_task_;
  kobuki_fleet_msgs::Task secondary_task_;
  kobuki_fleet_msgs::TaskList* taskList_;///< storage for current newTaskList
  kobuki_fleet_msgs::HeartBeatList* heartBeatList_;  ///< buffer for storing the current heartbeatlist

  bool available_;

  std::vector<kobuki_fleet_msgs::LocationIdentifier> allLocalizations_;



  //TaskAction* task_manager_action_;
};

#endif /* CONTROLLERTASKS_H_ */
