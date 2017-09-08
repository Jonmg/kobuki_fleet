/*
 * RvizGUI.cpp
 *
 *  Created on: 26.05.2017
 *      Author: basti
 */

#include "RvizGUI.h"

RvizGUI::RvizGUI()
{
  removeMarkerDuration_ = ros::Duration(20.0);
  disconectionDuration_ = ros::Duration(5.0);

  sub1_ = n_.subscribe("machineHB", 10, &RvizGUI::callBackMachineBackHeartBeat, this);
  // sub2_ = n_.subscribe("/heartbeat_list_robot_1", 1, callBackHeartBeatList);
  sub3_ = n_.subscribe("/HB", 10, &RvizGUI::callBackHeartBeat, this);
  pubMarkerArray_ = n_.advertise<visualization_msgs::MarkerArray>("markers", 30);


}

RvizGUI::~RvizGUI()
{
  // TODO Auto-generated destructor stub
}

void RvizGUI::run()
{
  ros::Rate r_(30);

  while (ros::ok())
    {
      ros::spinOnce();   //call spinOnce on the beginning of the loop to work with most current data

      //ACHTUNG HIER ÄNDERN!!!! wenn mehr dazu kommt - OHNE DAS GEHT GAR NICHTS VERDAMN IT! hahaha
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      marker.lifetime = ros::Duration();
      marker.color.a = 1.0;

      // TASK
      ///@todo: Iteratorchaos beseitigen!
      int id = 0;
      robots_marker_array.markers.clear();
      tasks_marker_array.markers.clear();
      for(unsigned int i=0; i<g_tasklist.tasks.size(); i++)
      {
        marker.ns = "task_marker";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = g_tasklist.tasks[i].machinePose.position.x;
        marker.pose.position.y = g_tasklist.tasks[i].machinePose.position.y;
        marker.pose.position.z = 0.075;
        marker.scale.x = 0.3;
        marker.scale.y = 0.3;
        marker.scale.z = 0.15;
        marker.color.g = 0.0f;
        marker.color.r = 0.0f;
        marker.color.b = 1.0f;
        tasks_marker_array.markers.push_back(marker);


        marker.ns = "tid_marker";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.z = .5;
        marker.scale.z = 0.3;
        std::string taskID(std::to_string(g_tasklist.tasks[i].tid));
        marker.text = "M-ID:" + taskID;//Result;
        tasks_marker_array.markers.push_back(marker);


        if (g_tasklist.tasks[i].taskStatus == kobuki_fleet_msgs::Task::WORKING)
        {
          marker.ns = "robots_id_marker";
          marker.id = id++;
          marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.z = 1;
          std::string taskRid1(std::to_string(g_tasklist.tasks[i].rid1));
          std::string taskRid2(std::to_string(g_tasklist.tasks[i].rid2));
          marker.text = "PRI:" + taskRid1 + " SEC:" + taskRid2;//Result;
          tasks_marker_array.markers.push_back(marker);
        }


//        marker.ns = "taskStatus_marker";
//        marker.id = id++;
//        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//        marker.action = visualization_msgs::Marker::ADD;
//        marker.pose.position.z = 1;
//        if(g_tasklist.tasks[i].taskStatus == kobuki_fleet_msgs::Task::OPEN)
//        {
//          marker.color.g = 0.0f;
//          marker.color.r = 0.0f;
//          marker.color.b = 1.0f;
//          marker.text = "OPEN";
//        }
//        else if(g_tasklist.tasks[i].taskStatus == kobuki_fleet_msgs::Task::WORKING)
//        {
//          marker.color.g = 0.0f;
//          marker.color.r = 0.0f;
//          marker.color.b = 1.0f;
//          marker.text = "WORKING";
//        }
//        else if(g_tasklist.tasks[i].taskStatus == kobuki_fleet_msgs::Task::FINISHED)
//        {
//          marker.color.g = 0.5f;
//          marker.color.r = 0.0f;
//          marker.color.b = 0.5f;
//          marker.text = "FINISHED";
//        }
//        else if(g_tasklist.tasks[i].taskStatus == kobuki_fleet_msgs::Task::ERROR)
//          {
//          marker.color.g = 0.0f;
//          marker.color.r = 1.0f;
//          marker.color.b = 0.0f;
//          marker.text = "ERROR";
//          }
//        else
//        {
//          marker.color.g = 0.0f;
//          marker.color.r = 0.0f;
//          marker.color.b = 0.0f;
//          marker.text = "_";
//        }

        tasks_marker_array.markers.push_back(marker);
      }


  //    if (robots_marker_array.markers.size() != g_tasklist.tasks.size())
  //      robots_marker_array.markers.resize(g_tasklist.tasks.size()*5);

      //ROBOT
          for(unsigned int i=0; i<g_heartbeatlist.heartBeatList.size(); i++)
          {
            marker.ns = "robotPoseMarker";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = g_heartbeatlist.heartBeatList[i].x;
            marker.pose.position.y = g_heartbeatlist.heartBeatList[i].y;
            marker.pose.position.z = 0.05;
            marker.scale.x = 0.3;
            marker.scale.y = 0.3;
            marker.scale.z = 0.15;
  //          if(g_heartbeatlist.heartBeatList[i].nchb == true)
  //          {
  //            marker.color.r = 0.0f;
  //            marker.color.g = 0.0f;
  //            marker.color.b = 0.0f;
  //          }
  //          else
  //          {
              marker.color.r = 0.1f;
              marker.color.g = 0.1f;
              marker.color.b = 0.1f;
  //          }

            robots_marker_array.markers.push_back(marker);


            marker.ns = "ridMarker";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.z = 0.5;
            marker.scale.z = 0.3;
//            if(g_heartbeatlist.heartBeatList[i].nchb == true)
//            {
//              marker.color.r = 1.0f;
//              marker.color.g = 0.0f;
//              marker.color.b = 0.0f;
//            }
//            else
//            {
              marker.color.r = 0.0f;
              marker.color.g = 0.6f;
              marker.color.b = 0.4f;
//            }
            std::string robotID(std::to_string(g_heartbeatlist.heartBeatList[i].rid));
            marker.text = "R-ID:" + robotID;//Result;

            robots_marker_array.markers.push_back(marker);


            marker.ns = "ridMarker";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.z = 1;

            if(g_heartbeatlist.heartBeatList[i].rob_status == kobuki_fleet_msgs::HeartBeat::OK && g_heartbeatlist.heartBeatList[i].nchb == true)
            {
              marker.color.r = 0.0f;
              marker.color.g = 0.0f;
              marker.color.b = 1.0f;
              marker.text = "NC";
            }
            else if (g_heartbeatlist.heartBeatList[i].rob_status == kobuki_fleet_msgs::HeartBeat::OK && g_heartbeatlist.heartBeatList[i].nchb == false)
            {
              marker.color.r = 0.0f;
              marker.color.g = 0.6f;
              marker.color.b = 0.4f;
              marker.text = "WLAN";
            }
            //          }
            else if (g_heartbeatlist.heartBeatList[i].rob_status == kobuki_fleet_msgs::HeartBeat::DISCONNECTED)
            {
              marker.color.r = 1.0f;
              marker.color.g = 0.0f;
              marker.color.b = 0.0f;
              marker.text = "DISCONNECTED";
            }

            robots_marker_array.markers.push_back(marker);


//            marker.ns = "robotStatus_marker";
//            marker.id = id++;
//            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
//            marker.action = visualization_msgs::Marker::ADD;
//            marker.pose.position.z = 0.7;
//            if(g_heartbeatlist.heartBeatList[i].rob_status == kobuki_fleet_msgs::HeartBeat::OK)
//            {
//              marker.color.r = 0.0f;
//              marker.color.g = 1.0f;
//              marker.color.b = 0.0f;
//              marker.text = "OK";
//            }
//            else
//            {
//              marker.color.r = 1.0f;
//              marker.color.g = 0.0f;
//              marker.color.b = 0.0f;
//              marker.text = "DISCONNECTED";
//            }
//
//            robots_marker_array.markers.push_back(marker);
  //          ROS_INFO_STREAM("robots_marker_array SIZE: " << robots_marker_array.markers.size());
          }


      pubMarkerArray_.publish(tasks_marker_array);
      pubMarkerArray_.publish(robots_marker_array);
      r_.sleep();
    }   //ros while ok loop
}



void RvizGUI::callBackTaskList(const kobuki_fleet_msgs::TaskListConstPtr& msg)
{
  for(unsigned int i = 0; i < msg->tasks.size(); i++)
    ROS_INFO_STREAM(" task message: " << msg->tasks[i]);
  g_tasklist = *msg;
}

void RvizGUI::callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList::ConstPtr& msg)
{
  for(unsigned int i = 0; i< msg->heartBeatList.size(); i++)
    ROS_INFO_STREAM(" heartbeat message: " << msg->heartBeatList[i]);
  g_heartbeatlist = *msg;
}


//missing the ability to check if new
void RvizGUI::callBackHeartBeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  //ROS_INFO("Received HeartBeat from robot id %d", msg->rid);
  if ((msg->x == 0.0) && (msg->y == 0.0))
      return;

  // search for multible heartbeats and keep just the new one
  for (unsigned int i = 0; i < g_heartbeatlist.heartBeatList.size(); i++)
  {
    if (now - g_heartbeatlist.heartBeatList[i].header.stamp >= removeMarkerDuration_)
    {
      ROS_INFO("Removed HeartBeat from robot id %d", g_heartbeatlist.heartBeatList.at(i).rid);
      g_heartbeatlist.heartBeatList.erase(g_heartbeatlist.heartBeatList.begin() + i);
      //g_heartbeatlist.heartBeatList[i].rob_status = kobuki_fleet_msgs::HeartBeat::DISCONNECTED;
    }
    else if ((g_heartbeatlist.heartBeatList[i].rob_status != kobuki_fleet_msgs::HeartBeat::DISCONNECTED)
        && (now - g_heartbeatlist.heartBeatList[i].header.stamp) >= disconectionDuration_)
    {
      //g_heartbeatlist.heartBeatList.erase(g_heartbeatlist.heartBeatList.begin() + i);
      g_heartbeatlist.heartBeatList[i].rob_status = kobuki_fleet_msgs::HeartBeat::DISCONNECTED;
    }


    if (g_heartbeatlist.heartBeatList.at(i).rid == msg->rid)
    {
      g_heartbeatlist.heartBeatList.erase(g_heartbeatlist.heartBeatList.begin() + i);
    }

  }

  g_heartbeatlist.heartBeatList.push_back(*msg);
  g_heartbeatlist.heartBeatList.back().header.stamp = now; // reset timestamp to avoid timesync problems
}


void RvizGUI::callBackMachineBackHeartBeat(const kobuki_fleet_msgs::TaskConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  //ROS_INFO("Received HeartBeat from Machine id %d", msg->tid);

  bool task_found = false;

  // search for multible heartbeats and keep just the new one
  for (unsigned int i = 0; i < g_tasklist.tasks.size(); i++)
  {
    if (now - g_tasklist.tasks[i].header.stamp >= removeMarkerDuration_)
    {
      g_tasklist.tasks.erase(g_tasklist.tasks.begin() + i);
      //g_heartbeatlist.heartBeatList[i].rob_status = kobuki_fleet_msgs::HeartBeat::DISCONNECTED;
    }

    if (g_tasklist.tasks.at(i).tid == msg->tid)
    {
      g_tasklist.tasks.erase(g_tasklist.tasks.begin() + i);
    }
  }

  g_tasklist.tasks.push_back(*msg);
  g_tasklist.tasks.back().header.stamp = now; // reset timestamp to avoid timesync problems

  for (unsigned int j = 0; j < h_tasklist.tasks.size(); j++)
  {
    if (h_tasklist.tasks[j].header.stamp == msg->header.stamp && h_tasklist.tasks[j].tid == msg->tid)
    {
      h_tasklist.tasks[j].taskStatus = msg->taskStatus;
      h_tasklist.tasks[j].rid1 = msg->rid1;
      h_tasklist.tasks[j].rid2 = msg->rid2;
      task_found = true;
      break;
    }
  }

  if (task_found == false)
  {
    h_tasklist.tasks.push_back(*msg);
  }

}

