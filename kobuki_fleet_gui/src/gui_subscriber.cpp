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
#include "ros/ros.h"
#include <ros/subscriber.h>
#include <ros/time.h>
#include <rosconsole/macros_generated.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
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

#include <QApplication>
#include "DisplayGUI.h"
#include <QDebug>

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


bool printInformation(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

kobuki_fleet_msgs::TaskList g_tasklist;  //global
kobuki_fleet_msgs::TaskList h_tasklist;	//history of tasks
kobuki_fleet_msgs::HeartBeatList g_heartbeatlist;  //global
kobuki_fleet_msgs::HeartBeatList h_heartbeatlist;  //history of heartbeats

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gui_subscriber");
  /*
  QApplication app(argc, argv);
  qDebug() << " constructor";
  Display_GUI w;// = new DisplayGUI();
  qDebug() << " show";
  w.show();
  qDebug() << " exec";

  return app.exec();
  */

  ros::NodeHandle n;
  ros::Rate r(0.5);
  ros::Subscriber sub1 = n.subscribe("machineHB", 10, callBackMachineBackHeartBeat);
  //ros::Subscriber sub2 = n.subscribe("/heartbeat_list_robot_1", 1, callBackHeartBeatList);
  ros::Subscriber sub3 = n.subscribe("/HB", 10, callBackHeartBeat);
  ros::Publisher pubMarkerArray = n.advertise<visualization_msgs::MarkerArray>("markers", 30);
  ros::ServiceServer service = n.advertiseService("print_taskinfo", printInformation);

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

  //ROS_INFO("Received HeartBeat from robot id %d", msg->rid);

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

  //ROS_INFO("Received HeartBeat from Machine id %d", msg->tid);

  bool task_found = false;

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

  for (unsigned int j = 0; j < h_tasklist.tasks.size(); j++)
  {
	  if (h_tasklist.tasks[j].header.stamp == msg->header.stamp && h_tasklist.tasks[j].tid == msg->tid)
	  {
		  h_tasklist.tasks[j].task_status = msg->task_status;
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

bool printInformation(std_srvs::Empty::Request  &req,
    std_srvs::Empty::Response &res)
{
	int machine_tasks [9] = {0};
	int mach_tasks_status [9][4] = {{0},{0},{0},{0}};
	int primary_robot_tasks [6] = {0};
	int secondary_robot_tasks [6] = {0};
	int total_mach_tasks = 0;
	int open_tasks = 0, working_tasks = 0, finished_tasks = 0, failed_tasks = 0;

	std::cout << std::endl << "Taskhistory:" << std::endl << std::endl;
	for (unsigned int k = 0; k < h_tasklist.tasks.size(); k++)
	{
		if (h_tasklist.tasks[k].task_status == 9)
		{
			ROS_INFO_STREAM("Initialization task");
		}
		ROS_INFO_STREAM("Task-ID: " << h_tasklist.tasks[k].tid << " Timestamp: "<< h_tasklist.tasks[k].header.stamp);
		//ROS_INFO_STREAM("Task-Status: " << std::to_string(h_tasklist.tasks[k].task_status));
		switch (h_tasklist.tasks[k].task_status)
		{
		case 0:
			ROS_INFO_STREAM("Task-Status: open");
			break;
		case 1:
			ROS_INFO_STREAM("Task-Status: working");
			break;
		case 2:
			ROS_INFO_STREAM("Task-Status: finished");
			break;
		case 3:
			ROS_INFO_STREAM("Task-Status: ERROR");
			break;
		default:
			break;
		}
		ROS_INFO_STREAM("Primary Robot: " << h_tasklist.tasks[k].rid1 << " Secondary Robot: " << h_tasklist.tasks[k].rid2);
		std::cout << std::endl;

		if (h_tasklist.tasks[k].task_status != 9)
		{
			switch (h_tasklist.tasks[k].tid)
			{
			case 1:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [0][0]++;
					break;
				case 1:
					mach_tasks_status [0][1]++;
					break;
				case 2:
					mach_tasks_status [0][2]++;
					break;
				case 3:
					mach_tasks_status [0][3]++;
					break;
				}
				break;
			case 2:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [1][0]++;
					break;
				case 1:
					mach_tasks_status [1][1]++;
					break;
				case 2:
					mach_tasks_status [1][2]++;
					break;
				case 3:
					mach_tasks_status [1][3]++;
					break;
				}
				break;
			case 3:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [2][0]++;
					break;
				case 1:
					mach_tasks_status [2][1]++;
					break;
				case 2:
					mach_tasks_status [2][2]++;
					break;
				case 3:
					mach_tasks_status [2][3]++;
					break;
				}
				break;
			case 4:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [3][0]++;
					break;
				case 1:
					mach_tasks_status [3][1]++;
					break;
				case 2:
					mach_tasks_status [3][2]++;
					break;
				case 3:
					mach_tasks_status [3][3]++;
					break;
				}
				break;
			case 5:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [4][0]++;
					break;
				case 1:
					mach_tasks_status [4][1]++;
					break;
				case 2:
					mach_tasks_status [4][2]++;
					break;
				case 3:
					mach_tasks_status [4][3]++;
					break;
				}
				break;
			case 6:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [5][0]++;
					break;
				case 1:
					mach_tasks_status [5][1]++;
					break;
				case 2:
					mach_tasks_status [5][2]++;
					break;
				case 3:
					mach_tasks_status [5][3]++;
					break;
				}
				break;
			case 7:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [6][0]++;
					break;
				case 1:
					mach_tasks_status [6][1]++;
					break;
				case 2:
					mach_tasks_status [6][2]++;
					break;
				case 3:
					mach_tasks_status [6][3]++;
					break;
				}
				break;
			case 8:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [7][0]++;
					break;
				case 1:
					mach_tasks_status [7][1]++;
					break;
				case 2:
					mach_tasks_status [7][2]++;
					break;
				case 3:
					mach_tasks_status [7][3]++;
					break;
				}
				break;
			case 9:
				switch (h_tasklist.tasks[k].task_status)
				{
				case 0:
					mach_tasks_status [8][0]++;
					break;
				case 1:
					mach_tasks_status [8][1]++;
					break;
				case 2:
					mach_tasks_status [8][2]++;
					break;
				case 3:
					mach_tasks_status [8][3]++;
					break;
				}
				break;
			default:
				break;
			}

			switch (h_tasklist.tasks[k].rid1)
			{
			case 0:
				primary_robot_tasks[0]++;
				break;
			case 1:
				primary_robot_tasks[1]++;
				break;
			case 2:
				primary_robot_tasks[2]++;
				break;
			case 3:
				primary_robot_tasks[3]++;
				break;
			case 4:
				primary_robot_tasks[4]++;
				break;
			case 5:
				primary_robot_tasks[5]++;
				break;
			default:
				break;
			}

			switch (h_tasklist.tasks[k].rid2)
			{
			case 0:
				secondary_robot_tasks[0]++;
				break;
			case 1:
				secondary_robot_tasks[1]++;
				break;
			case 2:
				secondary_robot_tasks[2]++;
				break;
			case 3:
				secondary_robot_tasks[3]++;
				break;
			case 4:
				secondary_robot_tasks[4]++;
				break;
			case 5:
				secondary_robot_tasks[5]++;
				break;
			default:
				break;
			}

			switch (h_tasklist.tasks[k].task_status)
			{
			case 0:
				open_tasks++;
				break;
			case 1:
				working_tasks++;
				break;
			case 2:
				finished_tasks++;
				break;
			case 3:
				failed_tasks++;
				break;
			default:
				break;
			}
		} 			//end-if
	} 				//end-for

	for (int i = 0; i < 9; i++)
	{
		for (int j=0; j<4; j++)
		{
			total_mach_tasks = total_mach_tasks + mach_tasks_status [i][j];
		}
	}				//counting the total number of tasks

	std::cout << "Statistics:" << std::endl << std::endl;

	std::cout << "Total number of tasks: " << total_mach_tasks << std::endl;
	std::cout << "Total number of open tasks: " << open_tasks << std::endl;
	std::cout << "Total number of working tasks: " << working_tasks << std::endl;
	std::cout << "Total number of finished tasks: " << finished_tasks << std::endl;
	std::cout << "Total number of failed tasks: " << failed_tasks << std:: endl << std::endl;

	for (int i=0; i<9; i++)
	{
		std::cout << "Machine " << i+1 << ":" << std::endl;
		std::cout << "open: " << mach_tasks_status[i][0] << std::endl;
		std::cout << "working: " << mach_tasks_status[i][1] << std::endl;
		std::cout << "finished: " << mach_tasks_status[i][2] << std::endl;
		std::cout << "failed: " << mach_tasks_status[i][3] << std::endl;
		std::cout << std::endl;
	}

	for (int j = 0; j < 6; j++)
	{
		std::cout << "Robot " << j << ":" << std::endl;
		std::cout << "Primary tasks: " << primary_robot_tasks[j] << " Secondary tasks: " << secondary_robot_tasks[j] << std::endl;
	}
	std::cout << std::endl;

	return true;
}
