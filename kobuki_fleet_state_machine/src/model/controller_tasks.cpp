/*
 * controllertasks.cpp
 *
 *  Created on: Mar 2, 2017
 *      Author: jon
 */

#include "controller_tasks.h"

ControllerTasks::ControllerTasks():
heartBeatList_(NULL)
{
  ros::NodeHandle prvNh("~");

  taskList_ = new kobuki_fleet_msgs::TaskList;

  std::string topicTaskList;
  std::string topicNewTask;
  std::string topicInitialTaskList;
  std::string topicGetPlan;
  std::string topicTaskAC;
  std::string topicHeartBeatList;
  std::string topicGetCurrentSubTask;
  std::string topicsetTaskStatusService;
//  std::string getAllInventoryTopic;


  prvNh.param<int>("robot_id", robot_id_, 1);
  prvNh.param<std::string>("robot_name", robotName_, "/robotName");
  prvNh.param<std::string>("topic_new_task", topicNewTask, "/new_task");
  prvNh.param<std::string>("get_plan_topic", topicGetPlan, "/move_base/NavfnROS/make_plan");
  prvNh.param<std::string>("topic_bidding", biddOfferTopic_, "/bidding_ws_");
  prvNh.param<std::string>("topic_assign_task", assignTopic_, "/assign_task_1");
  prvNh.param<std::string>("topic_task_status", taskStatusClientTopic_, "/task_status_ws_");
  prvNh.param<std::string>("tf_base_frame", _tfBaseFrame, "map");
  prvNh.param<std::string>("tf_robot_frame", _tfRobotFrame, "robot0");
  prvNh.param<std::string>("topic_task_action_server", topicTaskAC, "robot0/task_action_server");
  prvNh.param<std::string>("topic_heart_beat_list", topicHeartBeatList, "/heart_beat_list");
  prvNh.param<std::string>("topic_get_current_subTask_vector_server", topicGetCurrentSubTask, "/getCurrentSubTask");
  prvNh.param<std::string>("topic_set_task_status_server", topicsetTaskStatusService, "/task_status+robot_id_");
  prvNh.param<std::string>("get_all_inventory_data_topic", getAllInventoryTopic_, "/getAllInventoryData");

  subsNewTask_ = nh_.subscribe(topicNewTask, 1, &ControllerTasks::callBackNewTask, this);
  subsHeartBeatList_ = nh_.subscribe(topicHeartBeatList, 1, &ControllerTasks::callBackHeartBeatList, this);
  planSrvClient_ = nh_.serviceClient<nav_msgs::GetPlan>(topicGetPlan);
  taskStatusClient_ = nh_.serviceClient<kobuki_fleet_msgs::TaskStatus>("defined_later");
  assignmentServer_ = nh_.advertiseService(assignTopic_, &ControllerTasks::receiveAssignationServer, this);
  getCurrentSubTaskServer_ = nh_.advertiseService(topicGetCurrentSubTask, &ControllerTasks::getCurrentSubTaskServer, this);
  taskStatusServer_ = nh_.advertiseService(topicsetTaskStatusService, &ControllerTasks::setTaskStatusServer, this);

  _getAllInventoryData = nh_.serviceClient<kobuki_fleet_msgs::getAllInventoryData>(getAllInventoryTopic_);
  if(!_getAllInventoryData.waitForExistence(ros::Duration(5)))
    ROS_ERROR_STREAM("the service ’" << getAllInventoryTopic_ << "’ is not available");

  //start the taskAction
  //task_manager_action_ = new TaskAction(topicTaskAC);


  //initializations:
  primary_task_.rid1 = 99;
  primary_task_.rid2 = 99;
  secondary_task_.rid1 = 99;
  secondary_task_.rid2 = 99;
  available_ = true;
}

ControllerTasks::~ControllerTasks()
{
  delete taskList_;
  //delete task_manager_action_;
}


void ControllerTasks::run(void)
{
  //get locations form inventory
  kobuki_fleet_msgs::getAllInventoryData srv;
  srv.request.start.data = true;
  if(!_getAllInventoryData.waitForExistence(ros::Duration(5)))
      ROS_ERROR_STREAM("TC:  the service ’" << getAllInventoryTopic_ << "’ is not available");
  while(!_getAllInventoryData.call(srv))
  {
    ROS_ERROR("TC:  Inventory read data failed. Repeat the call");
    usleep(100000);
  }

  allLocalizations_.clear();
  for (unsigned int i=0; i<srv.response.locations.size();i++)
  {
    allLocalizations_.push_back(srv.response.locations.at(i));
  }
  ROS_INFO("TC:  Inventory data received correctly");

  ros::Rate r(10);
  while(ros::ok())
  {
    ros::spinOnce();   ///@todo: timer callback
    r.sleep();
  }
}


kobuki_fleet_msgs::Task* const ControllerTasks::task(const TaskOrder& order)
{
  kobuki_fleet_msgs::Task* taskPtr = NULL;
  if(order == PRIMARY && primary_task_.rid1 == robot_id_)
  {
    //    ROS_INFO_STREAM(robotName_  << ":found primary task");
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
      ROS_ERROR_STREAM("TC: Not Possible to Get the secondary task status from the machine! srvTopic: " << topic);
    else
    {
      if (secondary_task_.taskStatus != srv.response.taskStatus)
      {
        //ROS_INFO_STREAM(robotName_  << ": secondary task status has changed to status: " << srv.response.task_status );
        secondary_task_.taskStatus = srv.response.taskStatus;
        if (srv.response.taskStatus == kobuki_fleet_msgs::Task::FINISHED)
        {
          ROS_INFO_STREAM(robotName_  << ": secondary task already finished" );
          secondary_task_.rid1 = 99;
          secondary_task_.rid2 = 99;
          secondary_task_.tid = 99;
        }
      }
      else //if (srv.response.task_status == kobuki_fleet_msgs::Task::WORKING
        //||srv.response.task_status == kobuki_fleet_msgs::Task::ERROR
        //|| srv.response.task_status == kobuki_fleet_msgs::Task::OPEN)
      {
        //ROS_INFO_STREAM(robotName_  << ": found secondary task");
        taskPtr = &secondary_task_;
      }
    }
  }

  //if(!taskPtr)
  //  ROS_ERROR_STREAM_THROTTLE(2, robotName_  << ": no task found for  robot " << robot_id_);

  return taskPtr;
}

bool ControllerTasks::setTaskStatusServer(kobuki_fleet_msgs::TaskStatus::Request  &req,
    kobuki_fleet_msgs::TaskStatus::Response &res)
{
  std_msgs::UInt8 newState;
  std_msgs::UInt16 taskId;
  newState.data = req.taskStatus;
  taskId.data = req.tid;
  setTaskState(taskId, newState);
  res.taskStatus = true;
  return true;
}

bool  ControllerTasks::setTaskState(const std_msgs::UInt16& tid, std_msgs::UInt8& newState)
{
  //if task has finished, reset the primary task
  taskStatusClient_ = nh_.serviceClient<kobuki_fleet_msgs::AssignTask>(taskStatusClientTopic_);
  kobuki_fleet_msgs::TaskStatus srv;
  srv.request.action = srv.request.SET;
  srv.request.tid = tid.data;
  srv.request.rid =  robot_id_;
  srv.request.taskStatus = newState.data;
  std::string topic(taskStatusClientTopic_ + std::to_string(tid.data));
  taskStatusClient_ = nh_.serviceClient<kobuki_fleet_msgs::TaskStatus>(topic);
  if (!taskStatusClient_.call(srv))
    ROS_ERROR("TC: Not Possible to set the task to finished on the machine!");

  //change the info to send through the action
  /*task_manager_action_->setStatus(newState.data);*/
//  ROS_INFO_STREAM(robotName_  << ":TaskId: " << tid.data << " NewState: " << newState);
//  ROS_INFO_STREAM("TC: Primary_task_.tid: " << primary_task_.tid  << "  primary_task_.rid1: " << primary_task_.rid1);
//  ROS_INFO_STREAM("TC: Secondary_task_.tid: " << secondary_task_.tid  << "  secondary_task_.rid1: " << secondary_task_.rid1);
  if (tid.data == primary_task_.tid && robot_id_ == primary_task_.rid1)
  {
    ROS_INFO_STREAM(robotName_  << ":Actualize the primary task. New taskStatus: " << newState);
    if (newState.data == kobuki_fleet_msgs::Task::FINISHED)
    {
      primary_task_.tid = 99;
      primary_task_.rid1 = 99;
      primary_task_.tid = 99;
    }
    primary_task_.taskStatus = newState.data;
    return true;
  }
  else if (tid.data == secondary_task_.tid && robot_id_ == primary_task_.rid2)
  {
    ROS_INFO_STREAM(robotName_  << ":Actualize the secondary task. New taskStatus: " << newState);
    if (newState.data == kobuki_fleet_msgs::Task::FINISHED)
    {
      secondary_task_.rid1 = 99;
      secondary_task_.rid2 = 99;
      secondary_task_.tid = 99;
    }
    secondary_task_.taskStatus = newState.data;
    return true;
  }
  else
  {
    ROS_ERROR("TC: Neither Primary nor Secondary task find with this tid");
    return false;
  }
}


std_msgs::UInt8  ControllerTasks::getTaskState(const std_msgs::UInt16 tid)
{
  std_msgs::UInt8 status;
  status.data = 99;
  //prove if the task is in our taskList
  if(primary_task_.tid == tid.data)
  {
    status.data = primary_task_.taskStatus;
  }
  else  if(secondary_task_.tid == tid.data)
  {
    status.data = secondary_task_.taskStatus;
  }
  else
  {
    ROS_ERROR("TC: No primary or secondary taskID selected!");
  }
  return status;
}


bool ControllerTasks::receiveAssignationServer(kobuki_fleet_msgs::AssignTask::Request  &req,
    kobuki_fleet_msgs::AssignTask::Response &res)
{
  kobuki_fleet_msgs::Task tmpTask;
  for (unsigned int i=0; i< taskList_->tasks.size(); i++)
    {
      if(taskList_->tasks.at(i).tid == req.tid && taskList_->tasks.at(i).header.stamp == req.header.stamp)
      {
        tmpTask = taskList_->tasks.at(i);
        break;
      }
    }

  int assignmentType = req.taskType;
  //ROS_INFO_STREAM(robotName_  << ":Assignment type: " << assignmentType );

  if(assignmentType == 1)//req.PRIMARY
  {
    if (primary_task_.taskStatus == primary_task_.WORKING)
    {
      //No assignation is possible if we are running a primary task
      ROS_INFO_STREAM( robotName_  << ": No PRIMARY assignment is possible because we are already running a primary task");
      return false;
    }

    ROS_INFO_STREAM( robotName_  << ": Save primary task");
    primary_task_ = tmpTask;
    primary_task_.rid1 = robot_id_;
    return true;
  }
  //if it is a secondary task assignment and there is no secondary task already assigned
  else if((assignmentType == 2) && (secondary_task_.rid2 == 99))
  {
    ROS_INFO_STREAM( robotName_  << ": Save secondary task");
    secondary_task_ = tmpTask;
    secondary_task_.rid1 = req.rid1;
    secondary_task_.rid2 = req.rid2;
    return true;
  }
  else
  {
    ROS_WARN_STREAM( robotName_  << ": No SECONDARY assignment type could be possible!");
    return false;
  }
}

void ControllerTasks::callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList& msg)
{
  if(!heartBeatList_)
    heartBeatList_ = new kobuki_fleet_msgs::HeartBeatList(msg);
  else
    *heartBeatList_ = msg;
}

bool ControllerTasks::getCurrentSubTaskServer(kobuki_fleet_msgs::pop_subTaskVector::Request  &req,
    kobuki_fleet_msgs::pop_subTaskVector::Response &res)
{
  if (!available_)
  {
    available_ = true;
    ROS_INFO_STREAM(robotName_  << ": The robot is available again for new tasks");
  }

//  ROS_INFO_STREAM("TC: primary task: " << primary_task_ << "/n sencondary task: " << secondary_task_ );
  kobuki_fleet_msgs::Task* curPrimaryTask = task(PRIMARY);
  kobuki_fleet_msgs::Task* working = NULL;
  //	    ROS_INFO_STREAM("TC: Checking primary task");
  if(curPrimaryTask)
  {
//    ROS_INFO_STREAM(robotName_  << ": received primary task " << curPrimaryTask->tid << " for robot " << robot_id_);
    if(!(curPrimaryTask->taskStatus == curPrimaryTask->OPEN))  ///@todo process other task states..maybe unnecessary for primary task
    {
      //	        ROS_INFO_STREAM(robotName_  << ": Primary task already processed. Searching for secondary task");
      curPrimaryTask = NULL;  //set primary task to NULL so secondary is opened
    }
    else
    {
//      ROS_INFO_STREAM(robotName_  << ": Primary task OPENED and received");
      working = curPrimaryTask;
    }
  }

  kobuki_fleet_msgs::Task* curSecondaryTask = task(SECONDARY);
  //	    ROS_INFO_STREAM( "Checking secondary task");
  if(!curPrimaryTask && curSecondaryTask)
  {
//    ROS_INFO_STREAM(robotName_  << ": received secondary task " << curSecondaryTask->tid);

    //if FINISHED
    if(curSecondaryTask->taskStatus == kobuki_fleet_msgs::Task::FINISHED)
    {
      //	    		ROS_INFO_STREAM(robotName_  << ": Secondary task already finished");
    }
    //if ERROR -> adopt the task
    else if(curSecondaryTask->taskStatus == kobuki_fleet_msgs::Task::ERROR)
    {
      ROS_INFO_STREAM(robotName_  << ": found failed secondary task with id " << curSecondaryTask->tid
          << ". Start processing this task");
      working = curSecondaryTask;
    }
    //task is still open, check heartbeat of robot
    else if((curSecondaryTask->taskStatus == kobuki_fleet_msgs::Task::OPEN) || (curSecondaryTask->taskStatus == kobuki_fleet_msgs::Task::WORKING))
    {
      std_msgs::UInt16 id;
      id.data = curSecondaryTask->rid1;

      for(auto iter = heartBeatList_->heartBeatList.begin(); iter < heartBeatList_->heartBeatList.end(); iter++)
      {
        if(iter->rid == curSecondaryTask->rid1)
        {
          //	    				ROS_INFO_STREAM(robotName_  << ": found robot with id" << id.data << std::endl);

          if(iter->rob_status == kobuki_fleet_msgs::HeartBeat::DISCONNECTED)
          {
            ROS_INFO_STREAM(robotName_  << ": robot " << id.data << " is not responding. The task will be overtaken. TaskId:  " << curSecondaryTask->tid);
            working = curSecondaryTask;
          }
          else
            working = NULL;

          break;
        }
        else if (iter == heartBeatList_->heartBeatList.end())
        {
          ROS_ERROR_STREAM(robotName_  << ": error! heart beat of robot " << id.data << " could not be found. The task will be overtaken. TaskId:  " << curSecondaryTask->tid);
          working = curSecondaryTask;
        }
      }
    }
    else
    {
      ROS_ERROR_STREAM(robotName_  << ": secondary task has a state (not implemented yet) " << static_cast<unsigned int>(curSecondaryTask->taskStatus));
      working = NULL;
    }
  }


  if (!working)
  {
//    	    	ROS_INFO_STREAM_THROTTLE(3, robotName_  << ":No task to process. ");
    //res.currentTask = *working;
    return true;

  }

  //create the subTaskVector
  kobuki_fleet_msgs::SubTaskVector subTaskVector;
  subTaskVector.subtasks.clear();
  createBNTsubtasks(*working, &subTaskVector);
  ROS_INFO_STREAM("TC: Controller_task_node: Created SubtaskVector with : " << subTaskVector.subtasks.size() << " subtasks");


//  ROS_INFO_STREAM(robotName_  << ": will set task " << working->tid << " to state working ");
  std_msgs::UInt8 newState;
  newState.data = kobuki_fleet_msgs::Task::WORKING;
  std_msgs::UInt16 taskId;
  taskId.data = working->tid;
  if(!setTaskState(taskId, newState))
  {
    ROS_ERROR_STREAM(robotName_  << ": error! Changing status of state " << working->tid << " failed ");
  }

  ROS_INFO_STREAM("TC: sending new task and subtask to stateMachine. Robot is no longer available until task finishes");
  available_ = false;
  res.currentTask = *working;
  res.subTasks = subTaskVector;
  res.status.data = true;
  //model_->setCurentTask(*working);
  return true;
}

void ControllerTasks::callBackNewTask(const kobuki_fleet_msgs::NewTask& msg)
{
  //no accept new tasks if it is running one
  if(!available_)
    return;

  //1- Check if the task is already in our list
  for (unsigned int i=0; i< taskList_->tasks.size(); i++)
  {
    if(taskList_->tasks.at(i).tid == msg.tid && taskList_->tasks.at(i).header.stamp == msg.header.stamp)
    {
      return;
    }
  }
  ROS_INFO_STREAM( robotName_  << ": new task received");

  kobuki_fleet_msgs::Task task;
  task.header = msg.header;
  task.machineName = msg.machineName;
  task.tid = msg.tid;
  task.materialType = msg.materialType;


  int cost =0;
  //Here the cost calculation should depend on the task type. Right now only oneType
  if(msg.taskType == 1)
    cost = findCostsSrc2M(msg.machineName, msg.materialType, &task);
//  else if (msg.taskType == 2)
//    cost = findCosts(msg.machineName, msg.materialType, &task);

  if(cost == -1)
  {
    ROS_WARN_STREAM( robotName_  << ":Calculated costs are 0, something went wrong");
    return;
  }
  taskList_->tasks.push_back(task);

  ROS_INFO_STREAM( robotName_  << ":Source elected: " << task.srcName << "  finalCos: " << cost);

  //3- call the machine server to send the cost
  kobuki_fleet_msgs::BiddingOffer offer;
  offer.request.machineName = msg.machineName;
  offer.request.robotName = robotName_;
  offer.request.bidd.header.stamp = ros::Time::now();
  offer.request.bidd.cost = cost;
  offer.request.bidd.tid = msg.tid;
  offer.request.bidd.rid = robot_id_;
  offer.request.cancelOffer = false;
  std::string biddTopic = (biddOfferTopic_ + std::to_string(msg.tid));
  biddingClient_ = nh_.serviceClient<kobuki_fleet_msgs::BiddingOffer>(biddTopic);

  //ROS_INFO_STREAM("TC: the offer topic is:" << biddTopic);

  if(!biddingClient_.waitForExistence(ros::Duration(2)))
    ROS_WARN_STREAM("TC: bidding server is not available");

  while(!biddingClient_.call(offer))
  {
    ROS_ERROR_STREAM("TC: The bidding offer call with topic " << biddTopic << " could not be called");
    usleep(100000);
  }
  if(!offer.response.success)
    ROS_WARN_STREAM("TC: The bidding offer call with topic " << biddTopic << " was unsuccessful");
}

int ControllerTasks::findCostsSrc2M(std::string machineFileName, int materialType, kobuki_fleet_msgs::Task *task)
{
  int cost = 0;

  //2- find the path to the wsPose
  //2.1 find the robotPose
  tf::StampedTransform tf;
  geometry_msgs::Pose Robotpose;
  try
  {
    _listener.lookupTransform(_tfBaseFrame, _tfRobotFrame, ros::Time(0), tf);
    Robotpose.position.x = tf.getOrigin().x();
    Robotpose.position.y = tf.getOrigin().y();
    Robotpose.orientation.w = tf.getRotation().getW();
    Robotpose.orientation.x = tf.getRotation().getX();
    Robotpose.orientation.y = tf.getRotation().getY();
    Robotpose.orientation.z = tf.getRotation().getZ();
  }
  catch(tf::TransformException& e)
  {
    ROS_ERROR_STREAM("TC:  : Error calling tfRobotFrame" << e.what());
    return -1;
  }

  //2.2 find machine and sources pose
  geometry_msgs::Pose machinePose;
  std::vector<kobuki_fleet_msgs::LocationIdentifier> posibleSources;
  for (unsigned int i=0;i<allLocalizations_.size();i++)
  {
    //find Machine Pose
    if(allLocalizations_.at(i).description.data == machineFileName)
    {
      machinePose = allLocalizations_.at(i).pose;
    }

    //find Possible source wit material type XX
    if(allLocalizations_.at(i).materialType.data == materialType )
    {
      posibleSources.push_back(allLocalizations_.at(i));
    }

    if(i == allLocalizations_.size())
      ROS_ERROR_STREAM("TC:  MachineFileName not found in saved location vector ");
  }

  //2.3 find closes source and its pose
  int lowestCost2Source = 99999;
  kobuki_fleet_msgs::LocationIdentifier bestSource;
  for (unsigned int i=0;i<posibleSources.size();i++)
  {
    int tmpCost = findPath(Robotpose, posibleSources.at(i).pose);
//    ROS_INFO_STREAM( robotName_  << ": pathCost from Robotpose  to " << posibleSources.at(i).description.data << ": " << tmpCost);
    if (tmpCost < lowestCost2Source)
    {
      lowestCost2Source = tmpCost;
      bestSource = posibleSources.at(i);
    }
  }

  int srcToMachinecost = findPath(bestSource.pose, machinePose);
  task->srcName = bestSource.description.data;
  task->robotPose = Robotpose;
  task->machinePose = machinePose;
  task->srcPose = bestSource.pose;

  if (!lowestCost2Source || !srcToMachinecost)
    return -1;

  cost = lowestCost2Source + srcToMachinecost;
//  ROS_INFO_STREAM("TC:  The path costs are: Robot2Src: " << lowestCost2Source << " + Src2Machine: " << srcToMachinecost);

  return cost;

}

int ControllerTasks::findPath(geometry_msgs::Pose initPose, geometry_msgs::Pose finalPose)
{
  int path = 0;

  //3 Find path/Cost
  nav_msgs::GetPlan planner;
  planner.request.start.header.frame_id = "map";
  planner.request.goal.header.frame_id = "map";
  planner.request.start.pose = initPose;
  planner.request.goal.pose = finalPose;
  if(planSrvClient_.call(planner))
  {
    //ROS_INFO_STREAM( robotName_  << ": call to planner successful");
    if(!planner.response.plan.poses.size())
    {
      ROS_WARN_STREAM( robotName_  << ": No path could be found for /n initial pose:  " << initPose << "/n final pose: " << finalPose);
    }
    else
    {
//      ROS_INFO_STREAM(robotName_  << ":PlanTaskNode. path: " << planner.response.plan.poses.size());
      path = planner.response.plan.poses.size();
    }
  }
  else
  {
    ROS_ERROR("TC: could not call service make_plan from move_base");
  }

  return path;
}

void ControllerTasks::createBNTsubtasks (kobuki_fleet_msgs::Task task, kobuki_fleet_msgs::SubTaskVector *subTasks)
{
  kobuki_fleet_msgs::SubTask subTask;

  ros::Duration waitingTime(3);

  subTask.subTasktType = "M";
  subTask.serviceArea = task.srcName;
  subTask.poseNew = task.srcPose;
  subTasks->subtasks.push_back(subTask);

  subTask.subTasktType = "W";
  subTask.waitingTime = waitingTime;
  subTasks->subtasks.push_back(subTask);

  subTask.subTasktType = "M";
  subTask.serviceArea = task.machineName;
  subTask.poseNew = task.machinePose;
  subTasks->subtasks.push_back(subTask);

  subTask.subTasktType = "W";
  subTask.waitingTime = waitingTime;
  subTasks->subtasks.push_back(subTask);
}

