/*
 * MachineTaskManager.cpp
 *
 *  Created on: Feb 13, 2017
 *      Author: jon
 */

#include "MachineTaskManager.h"

// list::sort
#include <iostream>
#include <list>
#include <string>
#include <cctype>

bool compare_costs (const kobuki_fleet_msgs::Bidding& first, const kobuki_fleet_msgs::Bidding& second);

MachineTaskManager::MachineTaskManager():
            loop_rate_(5)
{
  ros::NodeHandle prvNh("~");

  double dVarX = 0;
  double dVarY = 0;
  int wsId;
  std::string topicPublishNewTaskService;
  std::string topicNewTask;
  std::string topicMachineTaskInfo;
  std::string topicBiddingService;
  std::string topicTaskStatusService;

  prvNh.param<int>("machine_id", wsId, 1);
  prvNh.param<double>("ws_pose_x", dVarX, 1.0);
  prvNh.param<double>("ws_pose_y", dVarY, 1.0);
  prvNh.param<std::string>("topic_create_new_task_server", topicPublishNewTaskService, "/createNewTaskServer");
  prvNh.param<std::string>("topic_new_task", topicNewTask, "/new_task");
  prvNh.param<std::string>("topic_machine_HB", topicMachineTaskInfo, "machineHB");
  prvNh.param<std::string>("topic_assign_task", asignTaskTopic_, "/assign_task_");
  prvNh.param<std::string>("topic_bidding", topicBiddingService, "/bidding_ws_");
  prvNh.param<std::string>("topic_task_status", topicTaskStatusService, "/task_status_ws_");

  newTask_.newTask.position.x = dVarX;
  newTask_.newTask.position.y = dVarY;
  newTask_.newTask.orientation.w = 1;
  newTask_.header.frame_id = "map";
  newTask_.tid = wsId;
  newTask_.task_status = 9;//initialization on a false value

  task_.task =  newTask_.newTask;
//  task_.task.position.x = dVarX;
//  task_.task.position.y = dVarY;
//  task_.task.orientation.w = 1;
  task_.header.frame_id = newTask_.header.frame_id;
  task_.tid = newTask_.tid;
  task_.task_status = 9;//initialization on a false value

  //  topicBiddingService = (topicBiddingService + std::to_string(wsId));
  newTaskPub_ = nh_.advertise<kobuki_fleet_msgs::NewTask>(topicNewTask, 1);
  machineStatusPub_ = nh_.advertise<kobuki_fleet_msgs::Task>(topicMachineTaskInfo, 1);
  publishNewTaskServer_ = nh_.advertiseService(topicPublishNewTaskService, &MachineTaskManager::pubishNewTaskServiceCallback, this);
  receiveBiddingsServer_ = nh_.advertiseService(topicBiddingService, &MachineTaskManager::receiveBiddingsServer, this);
  taskStatusServer_ = nh_.advertiseService(topicTaskStatusService, &MachineTaskManager::taskStatusServer, this);

//  aClient_ =   new actionlib::SimpleActionClient<kobuki_fleet_msgs::ManagerTaskAction> ("task_action_server", true);
//  if(!aClient_->waitForServer( ros::Duration(5)))
//    ROS_ERROR_STREAM("actionClient could not be started. service not found!");

  publishNewTask_ = false;
}

MachineTaskManager::~MachineTaskManager()
{
//  delete aClient_;
}

bool MachineTaskManager::receiveBiddingsServer(kobuki_fleet_msgs::BiddingOffer::Request  &req,
    kobuki_fleet_msgs::BiddingOffer::Response &res)
{
  //check if it is a new bidding for the current task
  for (auto it=biddingList_.begin(); it!=biddingList_.end(); ++it)
  {
    if (req.bidd.newTask.tid != newTask_.tid)
    {
      ROS_ERROR_STREAM("bidden task is not available in this machine!");
      return false;
    }
    else if (it->rid == req.bidd.rid)
    {
      if (req.cancelOffer)
      {
        ROS_WARN_STREAM("The bidding for robot " << it->rid << " has been canceled!!");
        return true;
      }
      else
      {
        ROS_WARN_STREAM("The bidding for robot " << it->rid << " was already added!!");
        return false;
      }
    }
  }

  ROS_INFO_STREAM("Received new bidding from robot: "  << req.bidd.rid << "  with a cost of: " << req.bidd.cost);

  //add the new bidding to the list
  biddingList_.push_back(req.bidd);
  return true;
}

bool MachineTaskManager::taskStatusServer(kobuki_fleet_msgs::TaskStatus::Request  &req,
    kobuki_fleet_msgs::TaskStatus::Response &res)
{
  if (req.tid == newTask_.tid)
  {
    //check action type
    if(req.action == req.SET)
    {
      //add a comparation if they are the same?
      newTask_.task_status = req.task_status;
      task_.task_status = req.task_status;
      int status = req.task_status;
      ROS_INFO_STREAM(__PRETTY_FUNCTION__  <<  " New task status: " << status);
      return true;
    }
    else if (req.action == req.GET)
    {
      res.task_status = newTask_.task_status;
      return true;
    }
    else
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "incorrect action type. req.action: " << req.action);
      return false;
    }
  }
  else
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << "incorrect task id. req.tid: " << req.tid);
    return false;
  }
}

void MachineTaskManager::decisionBiddingCallback(const ros::TimerEvent& event)
{
  publishNewTask_ = false;
  ROS_INFO_STREAM(__PRETTY_FUNCTION__  <<  "A decision will be taken. stop offering the task");

  if (!biddingList_.size())
  {
    ROS_INFO("No offer received. return");
    return;
  }
  int primaryID = 999;

  //code to take a decision and assign the Primary and Secondary robots to the task:
  biddingList_.sort(compare_costs);
  //biddingListCopy for the secondary task assignment
  std::list<kobuki_fleet_msgs::Bidding> tmpBiddingList = biddingList_;

  //Modify it to fulfill a Task type an not a newTask type.
  kobuki_fleet_msgs::AssignTask srv;
  srv.request.task.header = newTask_.header;
  srv.request.task.tid = newTask_.tid;
  srv.request.task.task = newTask_.newTask;
  srv.request.task.task_status = srv.request.task.OPEN;
  srv.request.task.rid1 = 999;
  srv.request.task.rid2 = 999;
  srv.request.taskType = srv.request.PRIMARY;
  std::string topic;

  task_.rid1 = 999;
  task_.rid2 = 999;

  //NoteJon:add a waitForService??
  //if size==1n the call will be done anyway because it is before
  while(biddingList_.size())
  {
    srv.request.task.rid1 = biddingList_.front().rid;
    topic = asignTaskTopic_ + std::to_string(biddingList_.front().rid);
    sendAssignedTaskClient_ = nh_.serviceClient<kobuki_fleet_msgs::AssignTask>(topic);
    //ROS_INFO_STREAM(" PRYMARY assignment topic: " << topic);
    if(sendAssignedTaskClient_.call(srv))
    {
      primaryID = biddingList_.front().rid;
      ROS_INFO_STREAM(" PRYMARY task " << newTask_.tid << " assigned to robot: " << primaryID);
      //save the changes in the task_
      task_.rid1 = biddingList_.front().rid;

      //start the action server:
      /*kobuki_fleet_msgs::ManagerTaskGoal goal;
      goal.newTask = newTask_;
      goal.taskType = 1;
      aClient_->sendGoal(goal);*/
      break;
    }
    //ROS_ERROR_STREAM("not possible to assign PRIMARY task to robot: " << biddingList_.front().rid);
    biddingList_.pop_front();
  }

  if (!biddingList_.size())
    ROS_ERROR_STREAM("not possible to assign PRIMARY task to any robot");

  //Assign the secondary robot
  srv.request.taskType = srv.request.SECONDARY;
  srv.request.task.rid1 = primaryID;
  while(tmpBiddingList.size())
  {
    if (tmpBiddingList.front().rid == primaryID)
    {
      tmpBiddingList.pop_front();
      continue;
    }
    srv.request.task.rid2 = tmpBiddingList.front().rid;
    topic = asignTaskTopic_ + std::to_string(tmpBiddingList.front().rid);
    //ROS_INFO_STREAM(" SECONDARY assignment topic: " << topic);
    sendAssignedTaskClient_ = nh_.serviceClient<kobuki_fleet_msgs::AssignTask>(topic);
    if(sendAssignedTaskClient_.call(srv))
    {
      ROS_INFO_STREAM(" SECONDARY task " << newTask_.tid << " assigned to robot: " << tmpBiddingList.front().rid);
      task_.rid2 = tmpBiddingList.front().rid;
      break;
    }
    //ROS_ERROR_STREAM("not possible to assig PRIMARY task to robot: " << biddingList_.front().rid);
    tmpBiddingList.pop_front();
  }

  if (!tmpBiddingList.size())
    ROS_ERROR_STREAM("not possible to assign SECONDARY task to any robot");
  //  else
  //    ROS_INFO_STREAM(__PRETTY_FUNCTION__  << " a decision has been taken. SECONDARY elected robot: " << tmpBiddingList.front().rid );

}

bool MachineTaskManager::pubishNewTaskServiceCallback(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res)
{
  biddingList_.clear();
  newTask_.header.stamp = ros::Time::now();
  newTask_.task_status = newTask_.OPEN;

  task_.header.stamp = ros::Time::now();
  task_.task_status = task_.OPEN;

  //Note: period by parameter in the future
  timer1_ = nh_.createTimer(ros::Duration(2), &MachineTaskManager::decisionBiddingCallback, this, true);
  //activate the timer and publish the newTask until the timer is over.
  publishNewTask_ = true;

  ROS_INFO_STREAM(__PRETTY_FUNCTION__  << " start publishing a new task!");

  return true;
}

void MachineTaskManager::run()
{
  while(ros::ok())
  {
    ros::spinOnce();

    if(machineStatusPub_.getNumSubscribers())
    {
      machineStatusPub_.publish(task_);
    }

    if(publishNewTask_)
    {
      newTaskPub_.publish(newTask_);
    }
    loop_rate_.sleep();
  }
}

// comparison between the costs in the biddingOffers. Used to order the list
bool compare_costs (const kobuki_fleet_msgs::Bidding& first, const kobuki_fleet_msgs::Bidding& second)
{
  if (first.cost <= second.cost) return true;
  else return false;
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "machine_task_manager_node");

  MachineTaskManager machine;
  machine.run();

  return 0;
}


