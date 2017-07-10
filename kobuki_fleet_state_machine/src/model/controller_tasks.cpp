/*
 * controllertasks.cpp
 *
 *  Created on: Mar 2, 2017
 *      Author: jon
 */

#include "controller_tasks.h"

namespace bobbyrob
{

ControllerTasks::ControllerTasks(ros::NodeHandle& nh):
    nh_(nh)
{
    newTaskList_ = new kobuki_fleet_msgs::NewTaskList;

    std::string topicTaskList;
    std::string topicNewTask;
    std::string topicInitialTaskList;
    std::string topicGetPlan;
    std::string topicTaskAC;

    ros::NodeHandle prvNh("~");
    prvNh.param<int>("robot_id", robot_id_, 1);
    prvNh.param<std::string>("topic_new_task", topicNewTask, "/new_task");
    prvNh.param<std::string>("get_plan_topic", topicGetPlan, "/move_base/NavfnROS/make_plan");
    prvNh.param<std::string>("topic_bidding", biddOfferTopic_, "/bidding_ws_");
    prvNh.param<std::string>("topic_assign_task", assignTopic_, "/assign_task_1");
    prvNh.param<std::string>("topic_task_status", taskStatusClientTopic_, "/task_status_ws_");
    prvNh.param<std::string>("tf_base_frame", _tfBaseFrame, "map");
    prvNh.param<std::string>("tf_robot_frame", _tfRobotFrame, "robot0");
    prvNh.param<std::string>("topic_task_action_server", topicTaskAC, "robot0/task_action_server");

    subsNewTask_ = nh_.subscribe(topicNewTask, 1, &ControllerTasks::callBackNewTask, this);
    planSrvClient_ = nh_.serviceClient<nav_msgs::GetPlan>(topicGetPlan);
    taskStatusClient_ = nh_.serviceClient<kobuki_fleet_msgs::TaskStatus>("defined_later");
    assignmentServer_ = nh_.advertiseService(assignTopic_, &ControllerTasks::receiveAssignationServer, this);
    //start the taskAction
    //task_manager_action_ = new TaskAction(topicTaskAC);


    //initializations:
    primary_task_.rid1 = 999;
    primary_task_.rid2 = 999;
    secondary_task_.rid1 = 999;
    secondary_task_.rid2 = 999;
}

ControllerTasks::~ControllerTasks()
{
  delete newTaskList_;
  //delete task_manager_action_;
}

const kobuki_fleet_msgs::Task* const ControllerTasks::task(const TaskOrder& order)
{
  const kobuki_fleet_msgs::Task* taskPtr = NULL;
  if(order == PRIMARY && primary_task_.rid1 == robot_id_)
  {
    //ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "found primary task");
    taskPtr = &primary_task_;
  }
  else if(order == SECONDARY && secondary_task_.rid2 == robot_id_)
  {
    kobuki_fleet_msgs::TaskStatus srv;
    srv.request.action = srv.request.GET;
    srv.request.tid = secondary_task_.tid;
    srv.request.rid =  robot_id_;
    std::string topic(taskStatusClientTopic_ + std::to_string(secondary_task_.tid));
    taskStatusClient_ = nh_.serviceClient<kobuki_fleet_msgs::TaskStatus>(topic);
    if (!taskStatusClient_.call(srv))
      ROS_ERROR_STREAM("Not Possible to Get the secondary task status from the machine! srvTopic: " << topic);
    else
    {
      if (secondary_task_.task_status != srv.response.task_status)
      {
        //ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " secondary task status has changed to status: " << srv.response.task_status );
        secondary_task_.task_status = srv.response.task_status;
        if (srv.response.task_status == kobuki_fleet_msgs::Task::FINISHED)
        {
          ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " secondary task already finished" );
          secondary_task_.rid1 = 999;
          secondary_task_.rid2 = 999;
          secondary_task_.tid = 999;
        }
      }
      else //if (srv.response.task_status == kobuki_fleet_msgs::Task::WORKING
        //||srv.response.task_status == kobuki_fleet_msgs::Task::ERROR
        //|| srv.response.task_status == kobuki_fleet_msgs::Task::OPEN)
      {
        //ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " found secondary task");
        taskPtr = &secondary_task_;
      }
    }
  }

  //if(!taskPtr)
  //  ROS_ERROR_STREAM_THROTTLE(2, __PRETTY_FUNCTION__ << " no task found for  robot " << robot_id_);

  //kobuki_fleet_msgs::Task& taskRef = task;
  return taskPtr;
}

bool  ControllerTasks::setTaskState(const std_msgs::UInt16& tid, std_msgs::UInt8& newState)
{
  //if task has finished, reset the primary task
  taskStatusClient_ = nh_.serviceClient<kobuki_fleet_msgs::AssignTask>(taskStatusClientTopic_);
  kobuki_fleet_msgs::TaskStatus srv;
  srv.request.action = srv.request.SET;
  srv.request.tid = tid.data;
  srv.request.rid =  robot_id_;
  srv.request.task_status = newState.data;
  std::string topic(taskStatusClientTopic_ + std::to_string(tid.data));
  taskStatusClient_ = nh_.serviceClient<kobuki_fleet_msgs::TaskStatus>(topic);
  if (!taskStatusClient_.call(srv))
    ROS_ERROR("Not Possible to set the task to finished on the machine!");

  //change the info to send throught the action
  /*task_manager_action_->setStatus(newState.data);*/
  if (tid.data == primary_task_.tid)
  {
    if (newState.data == kobuki_fleet_msgs::Task::FINISHED)
    {
      primary_task_.tid = 999;
      primary_task_.rid1 = 999;
      primary_task_.tid = 999;
    }
    primary_task_.task_status = newState.data;
    return true;
  }
  else if (tid.data == secondary_task_.tid)
  {
    if (newState.data == kobuki_fleet_msgs::Task::FINISHED)
    {
      secondary_task_.rid1 = 999;
      secondary_task_.rid2 = 999;
      secondary_task_.tid = 999;
    }
    secondary_task_.task_status = newState.data;
    return true;
  }
  else
  {
    ROS_ERROR("Neither Primary nor Secondary task find with this tid");
    return false;
  }
}


std_msgs::UInt8  ControllerTasks::getTaskState(const std_msgs::UInt16 tid)
{
  std_msgs::UInt8 status;
  status.data = 999;
  //prove if the task is in our taskList
    if(primary_task_.tid == tid.data)
    {
      status.data = primary_task_.task_status;
    }
    else  if(secondary_task_.tid == tid.data)
    {
      status.data = secondary_task_.task_status;
    }
    else
    {
      ROS_ERROR("No primary or secondary taskID selected!");
    }
    return status;
}

void ControllerTasks::callBackNewTask(const kobuki_fleet_msgs::NewTask& msg)
{
  //1- Check if the task is already in our list
  for (unsigned int i=0; i< newTaskList_->newTasks.size(); i++)
  {
    if(newTaskList_->newTasks.at(i).tid == msg.tid && newTaskList_->newTasks.at(i).header.stamp == msg.header.stamp)
    {
      return;
    }
  }
  ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " new task received");
  newTaskList_->newTasks.push_back(msg);

  //2- find the path to the wsPose
  tf::StampedTransform tf;
  geometry_msgs::Pose pose;
  try
  {
    _listener.lookupTransform(_tfBaseFrame, _tfRobotFrame, ros::Time(0), tf);
    pose.position.x = tf.getOrigin().x();
    pose.position.y = tf.getOrigin().y();
    pose.orientation.w = tf.getRotation().getW();
    pose.orientation.x = tf.getRotation().getX();
    pose.orientation.y = tf.getRotation().getY();
    pose.orientation.z = tf.getRotation().getZ();
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR_STREAM("controller_task_list: Error calling tf " << e.what());
  }


  nav_msgs::GetPlan planner;
  planner.request.start.header.frame_id = "map";
  planner.request.goal.header.frame_id = "map";
  planner.request.start.pose = pose;
  planner.request.goal.pose = msg.newTask;
  int cost = 0;
  if(planSrvClient_.call(planner))
  {
    //ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " call to planner successful");
    if(!planner.response.plan.poses.size())
    {
      ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " No path could be found for pose:  " << pose);
    }
    else
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "PlanTaskNode. cost: " << planner.response.plan.poses.size());
      cost = planner.response.plan.poses.size();
    }
  }
  else
  {
    ROS_ERROR("PlanTaskNode: could not call service make_plan from move_base");
  }

  //3- call the machine server to send the cost
  kobuki_fleet_msgs::BiddingOffer offer;
  //offer.request.bidd.header. =
  offer.request.bidd.cost = cost;
  offer.request.bidd.newTask = msg;
  offer.request.bidd.rid = robot_id_;

  std::string biddTopic = (biddOfferTopic_ + std::to_string(msg.tid));
  biddingClient_ = nh_.serviceClient<kobuki_fleet_msgs::BiddingOffer>(biddTopic);
  //ROS_INFO_STREAM("the offer topic is:" << biddTopic);

  if(!biddingClient_.call(offer))
    ROS_INFO_STREAM("The bidding offer call with topic " << biddTopic << " was unsuccessful");
}

bool ControllerTasks::receiveAssignationServer(kobuki_fleet_msgs::AssignTask::Request  &req,
      kobuki_fleet_msgs::AssignTask::Response &res)
{
  int assignmentType = req.taskType;
  //ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "Assignment type: " << assignmentType );

  if(assignmentType == 1)//req.PRIMARY
  {
    if (primary_task_.task_status == primary_task_.WORKING)
    {
      //No assignation is possible if we are running a primary task
      ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " No PRIMARY assignment is possible because we are already running a primary task");
      return false;
    }

    ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " Save primary task");
    primary_task_ = req.task;
    return true;
  }
  //if it is a secondary task assignment and there is no secondary task already assigned
  else if((assignmentType == 2) && (secondary_task_.rid2 == 999))
    {
      ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " Save secondary task");
      secondary_task_ = req.task;
      return true;
    }
  else
  {
    ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " No SECONDARY assignment type could be possible!");
    return false;
  }
}

} /* namespace bobbyrob */
