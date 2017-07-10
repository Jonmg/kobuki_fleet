/*
 * controller_task_list.cpp
 *
 *  Created on: Aug 2, 2016
 *      Author: phil
 *  Modified on 10.02.2017
 *      Author: Jon
 */

/**
 * @file controller_task_list.cpp
 * @brief contains implementation of class ControllerTaskList
 */

#include "controller_task_list.h"

/**
 * @namespace bobbyrob
 */
namespace bobbyrob
{

ControllerTaskList::ControllerTaskList(ros::NodeHandle& nh):
    nh_(nh),
    taskList_(NULL),
    lastTid_(NULL),
    lastTstate_(NULL)
{
  taskList_ = new kobuki_fleet_msgs::TaskList;
  newTaskList_ = new kobuki_fleet_msgs::NewTaskList;

  std::string topicTaskList;
  std::string topicNewTask;
  std::string topicInitialTaskList;
  std::string getPlanTopic;

  ros::NodeHandle prvNh("~");
  prvNh.param<int>("robot_id", robot_id_, 1);
  prvNh.param<std::string>("topic_task_list", topicTaskList, "/task_list");
  prvNh.param<std::string>("topic_initial_task_list", topicInitialTaskList, "/initial_task_list");
  prvNh.param<std::string>("topic_new_task", topicNewTask, "/new_task");
  prvNh.param<std::string>("get_plan_topic", getPlanTopic, "/move_base/NavfnROS/make_plan");
  prvNh.param<std::string>("topic_bidding", biddOfferTopic_, "/bidding_ws_");
  prvNh.param<std::string>("topic_assign_task", assignTopic_, "/assign_task_1");
  prvNh.param<std::string>("tf_base_frame", _tfBaseFrame, "map");
  prvNh.param<std::string>("tf_robot_frame", _tfRobotFrame, "robot0");

  subsTaskList_ = nh_.subscribe(topicTaskList, 1, &ControllerTaskList::callBackTaskList, this);
  subsNewTask_ = nh_.subscribe(topicNewTask, 1, &ControllerTaskList::callBackNewTask, this);
  subsInitialTaskList_ = nh.subscribe(topicInitialTaskList, 1, &ControllerTaskList::callBackInitialTaskList, this);
  pubTaskList_ = nh_.advertise<kobuki_fleet_msgs::TaskList>(topicTaskList, 1);
  planSrvClient_ = nh_.serviceClient<nav_msgs::GetPlan>(getPlanTopic);
  assignmentServer_ = nh_.advertiseService(assignTopic_, &ControllerTaskList::receiveAssignationServer, this);


  //initializations:
  primary_task_.rid1 = 9999;
  primary_task_.rid2 = 9999;
  secondary_task_.rid1 = 9999;
  secondary_task_.rid2 = 9999;

}

ControllerTaskList::~ControllerTaskList()
{
  delete taskList_;
  delete lastTid_;
  delete lastTstate_;
  delete newTaskList_;
}

/*const kobuki_fleet_msgs::Task* const ControllerTaskList::task(const std_msgs::UInt16& id, const TaskOrder& order)const
{
  const kobuki_fleet_msgs::Task* taskPtr = NULL;
  if(!taskList_)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error. No task list received yet" << std::endl);
    return NULL;
  }
  for(auto iter = taskList_->tasks.begin(); iter < taskList_->tasks.end(); iter++)
  {
    if((iter->rid1 == id.data) && (order == PRIMARY))
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "found primary task with id " << iter->tid << std::endl);
      taskPtr = &*iter;
      break;
    }
    if((iter->rid2 == id.data) && (order == SECONDARY))
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " found secondary task with id " << iter->tid << std::endl);
      taskPtr = &*iter;
      break;
    }
  }
  if(!taskPtr)
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " no task found for  robot " << id << std::endl);
  return taskPtr;
}*/

const kobuki_fleet_msgs::Task* const ControllerTaskList::task(const TaskOrder& order)const
{
  const kobuki_fleet_msgs::Task* taskPtr = NULL;

  if(primary_task_.rid1 != robot_id_ && secondary_task_.rid2 != robot_id_)
  {
    ROS_ERROR_STREAM_THROTTLE(1,__PRETTY_FUNCTION__ << " error. No task received yet");
    return NULL;
  }

    if(order == PRIMARY && primary_task_.rid1 == robot_id_)
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "found primary task");
      taskPtr = &primary_task_;
    }
    else if(order == SECONDARY && secondary_task_.rid2 == robot_id_)
    {
      ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " found secondary task");
      taskPtr = &secondary_task_;
    }

  if(!taskPtr)
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " no task found for  robot " << robot_id_);

  //kobuki_fleet_msgs::Task& taskRef = task;
  return taskPtr;
}

bool  ControllerTaskList::setTaskState(const std_msgs::UInt16& tid, std_msgs::UInt8& newState)
{
  //if task has finished, reset the primary task
  if (newState.data == kobuki_fleet_msgs::Task::FINISHED)
  {
    primary_task_.rid1 = 999;
    primary_task_.tid = 999;
  }
  //prove if the task is in our taskList
  if(primary_task_.tid == tid.data)
  {
    primary_task_.task_status = newState.data;
    return true;
  }
  else  if(secondary_task_.tid == tid.data)
  {
    secondary_task_.task_status = newState.data;
    return true;
  }
  else
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error. No task with id " << tid << " found");
    return false;
  }
}

std_msgs::UInt8  ControllerTaskList::getTaskState(const std_msgs::UInt16 tid)
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

void  ControllerTaskList::callBackTaskList(const kobuki_fleet_msgs::TaskList& msg)
{
  if(!taskList_)
    return;
  if(lastTid_)
  {
    std_msgs::UInt16 tid;
    kobuki_fleet_msgs::Task* const curTask = this->task(*lastTid_, *taskList_);
    if(!curTask)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Task with id " << lastTid_->data  << " not found in buffer (this should not happen) "<< std::endl);
      return;
    }

    if(curTask->task_status != lastTstate_->data)
    {
      ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " error! Task " << lastTid_->data << " status should have been " << lastTstate_->data
          << " but contains " << curTask->task_status << " change state again and republish");
      curTask->task_status = lastTstate_->data;
      pubTaskList_.publish(*taskList_);
    }
    else
      *taskList_ = msg;
  }
  else
    *taskList_ = msg;
}

void ControllerTaskList::callBackInitialTaskList(const kobuki_fleet_msgs::TaskList& msg)
{
  if(!taskList_)
  {
    *taskList_ = msg;
    //subsInitialTaskList_.shutdown();
  }
}

void ControllerTaskList::callBackNewTask(const kobuki_fleet_msgs::NewTask& msg)
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
    ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " call to planner successful");

    if(!planner.response.plan.poses.size())
    {
      ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " No path could be found from ");
      cost=0;
    }
    else
    {
      ROS_INFO_STREAM("PlanTaskNode. cost: " << planner.response.plan.poses.size());
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
  ROS_INFO_STREAM("the offer topic is:" << biddTopic);

  if(!biddingClient_.call(offer))
    ROS_INFO("The offer was unsuccsseful");
}

bool ControllerTaskList::receiveAssignationServer(kobuki_fleet_msgs::AssignTask::Request  &req,
      kobuki_fleet_msgs::AssignTask::Response &res)
{
  int assignmentType = req.taskType;
  ROS_INFO_STREAM(__PRETTY_FUNCTION__ << "Assignment type: " << assignmentType );

  if(assignmentType == 1)//req.PRIMARY
  {
    if (primary_task_.task_status == primary_task_.WORKING)
    {
      //No assignation is possible if we are running a primary task
      ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " No assignation is possible beacause we are already running a primary task");
      return false;
    }

    ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " Save primary task");
    primary_task_ = req.task;
    return true;
  }
  //if it is a secondary task assignment and there is no secondary task already assigned
  else if((assignmentType == 2) )//&& (secondary_task_.task_status != secondary_task_.OPEN)
    {
    ROS_INFO_STREAM( __PRETTY_FUNCTION__ << " Save secondary task");
      secondary_task_ = req.task;
      return true;
    }
  else
  {
    ROS_WARN_STREAM( __PRETTY_FUNCTION__ << " No assignemnt type could be possible!");
    return false;
  }
}



const kobuki_fleet_msgs::Task* const ControllerTaskList::task(const std_msgs::UInt16& tid, const kobuki_fleet_msgs::TaskList& taskList)
{
  if(!taskList.tasks.size())
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error! Received invalid task list");
    return NULL;
  }
  for(std::vector<kobuki_fleet_msgs::Task>::const_iterator iter = taskList.tasks.begin(); iter < taskList.tasks.end(); iter++)
  {
    if(iter->tid == tid.data)
      return &*iter;
  }
  return NULL;
}

kobuki_fleet_msgs::Task* const ControllerTaskList::task(const std_msgs::UInt16& tid, kobuki_fleet_msgs::TaskList& taskList)
{
  if(!taskList.tasks.size())
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error! Received invalid task list");
    return NULL;
  }
  for(std::vector<kobuki_fleet_msgs::Task>::iterator iter = taskList.tasks.begin(); iter < taskList.tasks.end(); iter++)
  {
    if(iter->tid == tid.data)
      return &*iter;
  }
  return NULL;
}

} /* namespace bobbyrob */
