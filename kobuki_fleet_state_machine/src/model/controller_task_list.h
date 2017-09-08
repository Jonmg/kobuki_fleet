/*
 * controller_task_list.h
 *
 *  Created on: Aug 2, 2016
 *      Author: phil
 */

#ifndef SRC_STATE_MACHINE_MODEL_CONTROLLER_TASK_LIST_H_
#define SRC_STATE_MACHINE_MODEL_CONTROLLER_TASK_LIST_H_

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include "kobuki_fleet_msgs/TaskList.h"
#include "kobuki_fleet_msgs/NewTask.h"
#include "kobuki_fleet_msgs/NewTaskList.h"
#include "kobuki_fleet_msgs/BiddingOffer.h"
#include <kobuki_fleet_msgs/AssignTask.h>

#include <nav_msgs/GetPlan.h>
#include "nav_msgs/Path.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include "model.h"

/**
 * @file controller_task_list.h
 * @brief contains declaration of class ControllerTaskList
 */

/**
 * @enum TaskOrder
 * @brief Enumeration type for order of tasks
 */
enum TaskOrder
{
  PRIMARY = 1,//!< PRIMARY
  SECONDARY = 2  //!< SECONDARY
};

/**
 * @class ControllerTaskList
 * @brief Class for managing the tasklist
 * Class for altering, reading and re publishing the task list. States
 * can access the controller and receive tasks
 */
class ControllerTaskList
{
public:
  /**
   * Constructor
   * @param nh ros::NodeHandle reference to the shared node handle of the state machine
   */
  ControllerTaskList(ros::NodeHandle& nh);
  /**
   * Destructor
   */
  virtual ~ControllerTaskList();
  /**
   * Getter. Get task from the task list for a specific robot
   * @brief Get robot specific task (use robot ID)
   * @param id unsigned int
   * @return ohm_kobuki_fleet::Task task or NULL in case no task with the id exists
   */
  const kobuki_fleet_msgs::Task* const task(const TaskOrder& order)const;
  /**
   * Set state of a spcific task to a new status. Method checks, if tid and rid match (to determine
   * wether the robot is allowed to perform actions on that tid).
   * @brief Set state of a specific task
   * @param tid std_msgs::UInt16 task id
   * @param rid std_msgs::UInt16 robot id
   * @param newState new state
   * @return true in case of success, false in case rid and tid do not match or tid does not exist
   */
  bool setTaskState(const std_msgs::UInt16& tid, std_msgs::UInt8& newState);

  std_msgs::UInt8  getTaskState(const std_msgs::UInt16 tid);
private:
  /**
   * ROS callBack method for initial task list. Stores the received list in a member buffer
   * @brief ROS callback method for task list
   * @param msg kobuki_fleet_msgs::TaskList
   */
  void callBackInitialTaskList(const kobuki_fleet_msgs::TaskList& msg);
  /**
   * ROS callBack method for a new task list. Stores the received task in a member buffer
   * @brief ROS callback method for new tasks
   * @param msg kobuki_fleet_msgs::NewTask
   */
  void callBackNewTask(const kobuki_fleet_msgs::NewTask& msg);


  bool receiveAssignationServer(kobuki_fleet_msgs::AssignTask::Request  &req,
                                kobuki_fleet_msgs::AssignTask::Response &res);
  /**
   * ROS callBack method for task list. Stores the received list in a member buffer
   * @brief ROS callback method for task list
   * @param msg kobuki_fleet_msgs::TaskList
   */
  void callBackTaskList(const kobuki_fleet_msgs::TaskList& msg);
  /**
   * @brief Method reading a specific task from a const tasklist message
   * @param tid std_msgs::UInt16 TaskId of the searched task
   * @param taskList kobuki_fleet_msgs::TaskList tasklist to read from
   * @return pointer to task or NULL in case of non existend task ID
   */
  const kobuki_fleet_msgs::Task* const task(const std_msgs::UInt16& tid, const kobuki_fleet_msgs::TaskList& taskList);
  /**
   * @brief Method reading a specific task from a const tasklist message
   * @param tid std_msgs::UInt16 TaskId of the searched task
   * @param taskList kobuki_fleet_msgs::TaskList tasklist to read from
   * @return point to task or NULL in case of non existend task ID
   */
  kobuki_fleet_msgs::Task* const task(const std_msgs::UInt16& tid, kobuki_fleet_msgs::TaskList& taskList);


  ros::NodeHandle& nh_;                      ///< reference to the main node handle
  ros::Subscriber subsInitialTaskList_;             ///< ROS subscriber object
  ros::Subscriber subsTaskList_;             ///< ROS subscriber object
  ros::Subscriber subsNewTask_;             ///< ROS subscriber to a newTask
  ros::Publisher pubTaskList_;               ///< ROS publisher object for republishing task lists
  ros::ServiceClient planSrvClient_;
  ros::ServiceClient biddingClient_;  //<bidd for a new task
  ros::ServiceServer assignmentServer_;  //<receive the assignation for a new task

  tf::TransformListener _listener;
  std::string _tfBaseFrame;
  std::string _tfRobotFrame;
  std::string biddOfferTopic_;
  std::string assignTopic_;
  int robot_id_;

  kobuki_fleet_msgs::Task primary_task_;
  kobuki_fleet_msgs::Task secondary_task_;
  kobuki_fleet_msgs::TaskList* taskList_; ///< storage for current task list
  kobuki_fleet_msgs::NewTaskList* newTaskList_;///< storage for current newTaskList
  std_msgs::UInt16* lastTid_;                ///< buffer for storing the id of the last changed task
  std_msgs::UInt8* lastTstate_;              ///< buffer for storing the state of the last changed task

  Model* model_;
};

#endif /* SRC_STATE_MACHINE_MODEL_CONTROLLER_TASK_LIST_H_ */
