/*
 * DisplayGUI.h
 *
 *  Created on: 15.05.2017
 *      Author: basti
 */

#ifndef KOBUKI_FLEET_GUI_SRC_DISPLAYGUI_H_
#define KOBUKI_FLEET_GUI_SRC_DISPLAYGUI_H_

#include <ros/ros.h>
#include <QMainWindow>
#include <sstream>
#include "std_msgs/String.h"
#include "ui_DisplayGUI.h"
#include <QTimer>
#include "kobuki_fleet_msgs/TaskList.h"
#include "kobuki_fleet_msgs/HeartBeat.h"
#include "kobuki_fleet_msgs/HeartBeatList.h"
#include <ros/time.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <vector>

namespace Ui {
class Display_GUI;
}


class Display_GUI: public QMainWindow
{
	Q_OBJECT

public:
	Display_GUI(QWidget *parent = 0);
	virtual ~Display_GUI();
	void callBackMachineBackHeartBeat(const kobuki_fleet_msgs::TaskConstPtr& msg);
  void callBackHeartBeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg);
	bool printInformation(void);
	void fillComboBoxMachines (void);
	void fillComboBoxRobots (void);

public slots:
  void callRos(void);
  void refreshInformation(const ros::TimerEvent& event);
  void createNewTask(void);
  void killRobot(void);
  void respawnRobot(void);


private:
	ros::NodeHandle _nh;
	ros::Publisher _chatter_pub;
	ros::Subscriber _sub1;
	ros::Subscriber _sub3;
	ros::ServiceClient _client1;
  ros::ServiceClient _client2;

  kobuki_fleet_msgs::TaskList g_tasklist;  //global
  kobuki_fleet_msgs::TaskList h_tasklist; //history of tasks
  kobuki_fleet_msgs::HeartBeatList g_heartbeatlist;  //global
  kobuki_fleet_msgs::HeartBeatList h_heartbeatlist;  //history of heartbeats

  ros::Duration removeMarkerDuration_;
  ros::Duration disconectionDuration_;

  int _total_tasks;
  int _mach_tasks_status [9][4];
  int _primary_robot_tasks [6];
  int _secondary_robot_tasks [6];
  int _working_tasks, _open_tasks, _finished_tasks, _failed_tasks;
  int _chosen_machine;
  int _chosen_robot;
  int _machine_total_tasks;
  int _robot_total_tasks;
  int _chosen_machine_test;

	Ui::GUI* _guiui;
	QTimer _timerMain;

	ros::Timer _refreshTimer;
};


#endif /* KOBUKI_FLEET_GUI_SRC_DISPLAYGUI_H_ */
