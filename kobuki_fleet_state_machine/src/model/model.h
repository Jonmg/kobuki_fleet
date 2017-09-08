/*
 * model.h
 *
 *  Created on: Aug 30, 2016
 *      Author: Jon Martin
 */


/**
 * @file model.h
 * @brief declaration of class model
 */

#ifndef SRC_STATE_MACHINE_MODEL_MODEL_H_
#define SRC_STATE_MACHINE_MODEL_MODEL_H_

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>
#include <kobuki_fleet_msgs/Task.h>
#include "kobuki_fleet_msgs/SubTask.h"
#include "kobuki_fleet_msgs/SubTaskVector.h"

#include "Agent.h"

#include "kobuki_fleet_msgs/LocationIdentifier.h"

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
 * @class Model
 * @brief Data container for state machine
 * Class used as a data exchange container used by all states.
 */
class Model
{
public:
  /**
   * Constructor
   * @param robotId unique id of the robot within the robot fleet
   * @param nh reference to main node handle of the state machine
   */
  Model(const std_msgs::UInt16& robotId, ros::NodeHandle* nh);
  /**
   * Destructor
   */
  virtual ~Model();

  /**
   * @brief Getter. Get robotId
   * @return std_mgs::UInt16 reference to the robot id
   */
  const std_msgs::UInt16& robotId(void)const{return robotId_;}

  kobuki_fleet_msgs::Task const curTask(void)const;

  bool setCurentTask(kobuki_fleet_msgs::Task curTask);
  
  void setSimulation(bool simulation){simulation_= simulation;}
  
  bool checkSimulation(void){return simulation_;}

//  bool setUpMoveBase(void);

  ros::NodeHandle* const nodeHandle(void)const{return nh_;}

  /**
   * set inventory locations.
   * @param inventoyLocations
   */
  void setInventorySources(const kobuki_fleet_msgs::LocationIdentifier inventoyLocations);

  /**
   * Get inventory locations.
   * @param inventoyLocations
   * @return success
   */
  bool getInventorySources(kobuki_fleet_msgs::LocationIdentifier& inventoryLocations);

  /**
     * set subTasks.
     * @param SubTaskVector
     */
  void addSubTasks(const kobuki_fleet_msgs::SubTaskVector SubTaskVector);

  /**
   * Get subTasks.
   * @param SubTaskVector
   * @return success
   */
  bool getSubTasks(kobuki_fleet_msgs::SubTaskVector& SubTaskVector);

  /**
   * Set the Actual subTask
   */
  void setActualSubTask(kobuki_fleet_msgs::SubTask actualSubTask);

  /**
   * Get the Actual subTask
   */
  void getActualSubTask(kobuki_fleet_msgs::SubTask& actualSubTask);



private:
  ros::NodeHandle* const nh_;

  const std_msgs::UInt16 robotId_;          ///< unique robot id
  kobuki_fleet_msgs::Task curTask_;
  kobuki_fleet_msgs::LocationIdentifier inventoyLocations_;
  bool simulation_;
  kobuki_fleet_msgs::SubTaskVector _subTaskVector;
  kobuki_fleet_msgs::SubTask _actualSubTask;

};

#endif /* SRC_STATE_MACHINE_MODEL_MODEL_H_ */
