/*
 * MachineTaskManager.h
 *
 *  Created on: Feb 13, 2017
 *      Author: jon
 */

#ifndef MACHINETASKMANAGER_H_
#define MACHINETASKMANAGER_H_

#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Pose.h"
#include "kobuki_fleet_msgs/NewTask.h"
#include <kobuki_fleet_msgs/Bidding.h>
#include <kobuki_fleet_msgs/BiddingOffer.h>
#include <kobuki_fleet_msgs/AssignTask.h>
#include <kobuki_fleet_msgs/TaskStatus.h>
#include <kobuki_fleet_msgs/Task.h>
#include <std_srvs/Empty.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <kobuki_fleet_msgs/ManagerTaskAction.h>

class MachineTaskManager
{
public:
  MachineTaskManager();
  virtual ~MachineTaskManager();

  /*
   * Service server that allow the robots to bid for a determined task.
   * the biddingOffers are saved into a List
   */
  bool receiveBiddingsServer(kobuki_fleet_msgs::BiddingOffer::Request  &req,
                             kobuki_fleet_msgs::BiddingOffer::Response &res);

  /*
   * service to read or set a determined task status
   * @param req.tid
   * @param req.rid
   * @param req.action (GET=0 / SET=1)
   * @param req.task_status if GET: new status for task @tid
   * @param res.task_status if SET: response the status of @tid
   */
  bool taskStatusServer(kobuki_fleet_msgs::TaskStatus::Request  &req,
      kobuki_fleet_msgs::TaskStatus::Response &res);

  /*
   * Empty service to activate for a certain time the Machine task publisher.
   */
  bool pubishNewTaskServiceCallback(std_srvs::Empty::Request  &req,
                            std_srvs::Empty::Response &res);

  /*
   * it is activated with a "receiveBiddingsServer" call
   * orders the biddingOffersList and sends the decision to the Primary and Secondary Tasks
   */
  void decisionBiddingCallback(const ros::TimerEvent& event);

  void run();


private:
  ros::NodeHandle nh_;
  ros::Rate loop_rate_;

  ros::Publisher newTaskPub_;
  ros::Publisher machineStatusPub_;
  ros::ServiceServer publishNewTaskServer_;   ///Server to activate a newTask sending
  ros::ServiceServer receiveBiddingsServer_;  ///Server to receive the biddings from the robots
  ros::ServiceClient sendAssignedTaskClient_; ///Client to assig the task to a robot
  ros::ServiceServer taskStatusServer_;       ///Server to receive the status form the elected robots
  actionlib::SimpleActionClient<kobuki_fleet_msgs::ManagerTaskAction>* aClient_;

  ros::Time trigerTime_;
  geometry_msgs::Pose machinePose_;
  kobuki_fleet_msgs::NewTask newTask_;
  kobuki_fleet_msgs::Task task_;

  std::list<kobuki_fleet_msgs::Bidding> biddingList_;
  std::list<kobuki_fleet_msgs::Bidding>::iterator itBiddingList_;

  bool publishNewTask_;
  int wsId_;

  std::string asignTaskTopic_;

  ros::Timer timer1_;
};

#endif /* MACHINETASKMANAGER_H_ */
