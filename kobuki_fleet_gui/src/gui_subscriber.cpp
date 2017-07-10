/*
 * gui_subscriber.cpp
 *
 *  Created on: 31.07.2016
 *      Author: jasmin, Jon
 */
/**
 * @file gui_subscriber.cpp
 * @brief contains markers for gui
 */

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/rate.h>
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <iterator>
#include <string>
#include <vector>
#include "kobuki_fleet_msgs/TaskList.h"
#include "kobuki_fleet_msgs/HeartBeat.h"
#include "kobuki_fleet_msgs/HeartBeatList.h"
#include <resource_retriever/retriever.h>

ros::Duration removeMarkerDuration(30.0);
ros::Duration disconectionDuration(10.0);

/**
 * @brief ROS callback for task list message
 * @param msg ohm_kobuki_fleet::TaskList tasks
 * ROS Callback method to receive task lists
 */
void callBackTaskList(const kobuki_fleet_msgs::TaskListConstPtr& msg);

/**
 * @brief ROS callback for heartbeat list message
 * @param msg kobuki_fleet_msgs::HeartBeatList
 * ROS callback method to receive heartbeat lists
 */
void callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList::ConstPtr& msg);

/**
 * @brief calback function for heartbeat message from all robots
 * @param message container
 */
void callBackHeartBeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg);

/**
 * @brief calback function for machine heartbeat message from all machines
 * @param message container
 */
void callBackMachineBackHeartBeat(const kobuki_fleet_msgs::TaskConstPtr& msg);


kobuki_fleet_msgs::TaskList g_tasklist;  //global
kobuki_fleet_msgs::HeartBeatList g_heartbeatlist;  //global

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gui_subscriber");
  ros::NodeHandle n;
  ros::Rate r(0.5);
  ros::Subscriber sub1 = n.subscribe("machineHB", 10, callBackMachineBackHeartBeat);
  //ros::Subscriber sub2 = n.subscribe("/heartbeat_list_robot_1", 1, callBackHeartBeatList);
  ros::Subscriber sub3 = n.subscribe("/HB", 10, callBackHeartBeat);
  ros::Publisher pubMarkerArray = n.advertise<visualization_msgs::MarkerArray>("markers", 30);

  visualization_msgs::MarkerArray tasks_marker_array;
  visualization_msgs::MarkerArray robots_marker_array;

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
      marker.pose.position.x = g_tasklist.tasks[i].task.position.x;
      marker.pose.position.y = g_tasklist.tasks[i].task.position.y;
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
      marker.pose.position.z = 1.25;
      marker.scale.z = 0.3;
      std::string taskID(std::to_string(g_tasklist.tasks[i].tid));
      marker.text = "Task-ID: " + taskID;//Result;
      tasks_marker_array.markers.push_back(marker);

      marker.ns = "robots_id_marker";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.z = 1.5;
      std::string taskRid1(std::to_string(g_tasklist.tasks[i].rid1));
      std::string taskRid2(std::to_string(g_tasklist.tasks[i].rid2));
      marker.text = "Task-rid1:" + taskRid1 + " Task-rid2:" + taskRid2;//Result;
      tasks_marker_array.markers.push_back(marker);


      marker.ns = "taskStatus_marker";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.z = 1.75;
      if(g_tasklist.tasks[i].task_status == kobuki_fleet_msgs::Task::OPEN)
      {
        marker.color.g = 1.0f;
        marker.color.r = 1.0f;
        marker.color.b = 0.0f;
        marker.text = "OPEN";
      }
      else if(g_tasklist.tasks[i].task_status == kobuki_fleet_msgs::Task::WORKING)
      {
        marker.color.g = 1.0f;
        marker.color.r = 1.0f;
        marker.color.b = 0.0f;
        marker.text = "WORKING";
      }
      else if(g_tasklist.tasks[i].task_status == kobuki_fleet_msgs::Task::FINISHED)
      {
        marker.color.g = 1.0f;
        marker.color.r = 0.0f;
        marker.color.b = 0.0f;
        marker.text = "FINISHED";
      }
      else if(g_tasklist.tasks[i].task_status == kobuki_fleet_msgs::Task::ERROR)
        {
        marker.color.g = 0.0f;
        marker.color.r = 1.0f;
        marker.color.b = 0.0f;
        marker.text = "ERROR";
        }
      else
      {
        marker.color.g = 0.0f;
        marker.color.r = 0.0f;
        marker.color.b = 0.0f;
        marker.text = "_";
      }

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
          marker.scale.z = 0.1;
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
          marker.pose.position.z = 0.3;
          marker.scale.z = 0.2;
          if(g_heartbeatlist.heartBeatList[i].nchb == true)
          {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
          }
          else
          {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
          }
          std::string robotID(std::to_string(g_heartbeatlist.heartBeatList[i].rid));
          marker.text = "Robot-ID: " + robotID;//Result;

          robots_marker_array.markers.push_back(marker);


          marker.ns = "ridMarker";
          marker.id = id++;
          marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.z = 0.5;
          if(g_heartbeatlist.heartBeatList[i].nchb == true)
          {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.text = "NC Heartbeat";
          }
          else
          {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.text = "Wlan Heartbeat";
          }

          robots_marker_array.markers.push_back(marker);


          marker.ns = "robotStatus_marker";
          marker.id = id++;
          marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.z = 0.7;
          if(g_heartbeatlist.heartBeatList[i].rob_status == kobuki_fleet_msgs::HeartBeat::OK)
          {
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.text = "OK";
          }
          else
          {
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.text = "DISCONNECTED";
          }

          robots_marker_array.markers.push_back(marker);
//          ROS_INFO_STREAM("robots_marker_array SIZE: " << robots_marker_array.markers.size());
        }


    pubMarkerArray.publish(tasks_marker_array);
    pubMarkerArray.publish(robots_marker_array);
    r.sleep();
  }   //ros while ok loop
  ros::spin();
  return 0;
}

void callBackTaskList(const kobuki_fleet_msgs::TaskListConstPtr& msg)
{
  for(unsigned int i = 0; i < msg->tasks.size(); i++)
    ROS_INFO_STREAM(" task message: " << msg->tasks[i]);
  g_tasklist = *msg;
}

void callBackHeartBeatList(const kobuki_fleet_msgs::HeartBeatList::ConstPtr& msg)
{
  for(unsigned int i = 0; i< msg->heartBeatList.size(); i++)
    ROS_INFO_STREAM(" heartbeat message: " << msg->heartBeatList[i]);
  g_heartbeatlist = *msg;
}


//missing the ability to check if new
void callBackHeartBeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  ROS_INFO("Received HeartBeat from robot id %d", msg->rid);

  // search for multible heartbeats and keep just the new one
  for (unsigned int i = 0; i < g_heartbeatlist.heartBeatList.size(); i++)
  {
    if (now - g_heartbeatlist.heartBeatList[i].header.stamp >= removeMarkerDuration)
    {
      g_heartbeatlist.heartBeatList.erase(g_heartbeatlist.heartBeatList.begin() + i);
      //g_heartbeatlist.heartBeatList[i].rob_status = kobuki_fleet_msgs::HeartBeat::DISCONNECTED;
    }
    else if ((g_heartbeatlist.heartBeatList[i].rob_status != kobuki_fleet_msgs::HeartBeat::DISCONNECTED)
        && (now - g_heartbeatlist.heartBeatList[i].header.stamp) >= disconectionDuration)
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


void callBackMachineBackHeartBeat(const kobuki_fleet_msgs::TaskConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  ROS_INFO("Received HeartBeat from Machine id %d", msg->tid);

  // search for multible heartbeats and keep just the new one
  for (unsigned int i = 0; i < g_tasklist.tasks.size(); i++)
  {
    if (now - g_tasklist.tasks[i].header.stamp >= removeMarkerDuration)
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
}
