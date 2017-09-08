/*
 * DisplayGUI.cpp
 *
 *  Created on: 15.05.2017
 *      Author: basti
 */

#include "DisplayGUI.h"


Display_GUI::Display_GUI(QWidget *parent) :
	QMainWindow(parent),
	_guiui(new Ui::GUI)
{
  _chosen_machine = 0;
  _chosen_robot = 0;

	_guiui->setupUi(this);
	_chatter_pub = _nh.advertise<std_msgs::String>("hello", 100);
	_sub1 = _nh.subscribe("machineHB", 10, &Display_GUI::callBackMachineBackHeartBeat, this);
	_sub3 = _nh.subscribe("/HB", 10, &Display_GUI::callBackHeartBeat, this);

	connect(&_timerMain, SIGNAL(timeout(void)), this, SLOT(callRos(void)));
	connect(_guiui->pushButtonPublishTask, SIGNAL(clicked(void)), this, SLOT(createNewTask()));
	connect(_guiui->pushButtonRobotKill, SIGNAL(clicked(void)), this, SLOT(killRobot()));
	connect(_guiui->pushButtonRobotSpawn, SIGNAL(clicked(void)), this, SLOT(respawnRobot()));


	_timerMain.start(50);

	//*** Timer for auto-refresh of information ***
	_refreshTimer = _nh.createTimer(ros::Duration(0.5), &Display_GUI::refreshInformation, this);
//	g_tasklist.tasks.clear();
//	h_tasklist.tasks.clear();
//	g_heartbeatlist.heartBeatList.clear();
//	h_heartbeatlist.heartBeatList.clear();
}

Display_GUI::~Display_GUI() {
	// TODO Auto-generated destructor stub
//	delete ui;
}

void Display_GUI::fillComboBoxMachines()
{
  if(!h_tasklist.tasks.size())
    return;

  bool found = false;
  QString itemName;
  unsigned int itemToCompareWith;
  unsigned int newItem;
  unsigned int size = h_tasklist.tasks.size();
  int helperArray[size];
  int cache;
  int changed;

  /*** filling helperArray for sorting reasons ****/
  for (unsigned int i = 0; i < size; i++)
  {
    helperArray[i] = h_tasklist.tasks[i].tid;
  }

  //*** numerical sort of the entries of h_tasklist by TID ***
  do
  {
    changed = 0;

    for(unsigned int i = size - 1; i > 0; i--)
    {
      if(helperArray[i] < helperArray[i-1])
      {
        cache = helperArray[i];
        helperArray[i] = helperArray[i-1];
        helperArray[i-1] = cache;
        changed = 1;
      }
    }
  }while(changed);

  //*** transfer sorted entries into comboBoxMachines after checking if entry is already in comboBox ***
  for(unsigned int i = 0; i < size; i++)
  {
    for(int j = 0; j < _guiui->comboBoxMachines->count(); j++)
    {
      itemToCompareWith = _guiui->comboBoxMachines->itemText(j).toInt(0, 10);
      newItem = helperArray[i];
      if(newItem == itemToCompareWith)
      {
        found = true;
        break;
      }
    }
    if (found == false)
    {
      itemName = QString::number(helperArray[i]);
      _guiui->comboBoxMachines->addItem(itemName);
    }
  }
}

void Display_GUI::fillComboBoxRobots()
{
  if(!h_heartbeatlist.heartBeatList.size())
      return;

  bool found = false;
  QString itemName;
  unsigned int itemToCompareWith;
  unsigned int newItem;
  unsigned int size = h_heartbeatlist.heartBeatList.size();
  int helperArray[size];
  int cache;
  int changed;

  //*** filling helperArray for sorting reasons ***
  for (unsigned int i = 0; i < size; i++)
  {
    helperArray[i] = h_heartbeatlist.heartBeatList[i].rid;
  }

  //*** numerical sort of the entries of h_heartbeatlist by RID ***
  do
  {
    changed = 0;

    for(unsigned int i = size - 1; i > 0; i--)
    {
      if(helperArray[i] < helperArray[i-1])
      {
        cache = helperArray[i];
        helperArray[i] = helperArray[i-1];
        helperArray[i-1] = cache;
        changed = 1;
      }
    }
  }while(changed);

  //*** transfer sorted entries into comboBoxRobots after checking if entry is already in comboBox ***
  for(unsigned int i = 0; i < size; i++)
  {
    for(int j = 0; j < _guiui->comboBoxRobots->count(); j++)
    {
      itemToCompareWith = _guiui->comboBoxRobots->itemText(j).toInt(0, 10);
      newItem = helperArray[i];
      if(newItem == itemToCompareWith)
      {
        found = true;
        break;
      }
    }
    if (found == false)
    {
      itemName = QString::number(helperArray[i]);
      _guiui->comboBoxRobots->addItem(itemName);
    }
  }
}

void Display_GUI::callBackHeartBeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  bool robot_found = false;

  //ROS_INFO("Received HeartBeat from robot id %d", msg->rid);

  // search for multible heartbeats and keep just the new one
  for (unsigned int i = 0; i < g_heartbeatlist.heartBeatList.size(); i++)
  {
    if (now - g_heartbeatlist.heartBeatList[i].header.stamp >= removeMarkerDuration_)
    {

      g_heartbeatlist.heartBeatList.erase(g_heartbeatlist.heartBeatList.begin() + i);
      //g_heartbeatlist.heartBeatList[i].rob_status = kobuki_fleet_msgs::HeartBeat::DISCONNECTED;
    }
    if (g_heartbeatlist.heartBeatList.size() && g_heartbeatlist.heartBeatList.at(i).rid == msg->rid)
    {
      g_heartbeatlist.heartBeatList.erase(g_heartbeatlist.heartBeatList.begin() + i);
    }

  }

  g_heartbeatlist.heartBeatList.push_back(*msg);
  g_heartbeatlist.heartBeatList.back().header.stamp = now; // reset timestamp to avoid timesync problems

  for (unsigned int j = 0; j < h_heartbeatlist.heartBeatList.size(); j++)
    {
      if (h_heartbeatlist.heartBeatList[j].rid == msg->rid)
      {
        robot_found = true;
        break;
      }
    }

    if (robot_found == false)
    {
      h_heartbeatlist.heartBeatList.push_back(*msg);
    }
}

void Display_GUI::callBackMachineBackHeartBeat(const kobuki_fleet_msgs::TaskConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  //ROS_INFO("Received HeartBeat from Machine id %d", msg->tid);

  bool task_found = false;

  // search for multiple heartbeats and keep just the new one
  for (unsigned int i = 0; i < g_tasklist.tasks.size(); i++)
  {

    if (now - g_tasklist.tasks[i].header.stamp >= removeMarkerDuration_)
    {
      g_tasklist.tasks.erase(g_tasklist.tasks.begin() + i);
      //g_heartbeatlist.heartBeatList[i].rob_status = kobuki_fleet_msgs::HeartBeat::DISCONNECTED;
    }

    if (g_tasklist.tasks.size() && g_tasklist.tasks.at(i).tid == msg->tid)
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

void Display_GUI::respawnRobot()
{
  std::string respawnRobot("roslaunch kobuki_fleet_launch robot" + std::to_string(_chosen_robot) + ".launch &");
  system(respawnRobot.c_str());
}

void Display_GUI::killRobot()
{
  std::string topic_kill_robot = "/robot" + std::to_string(_chosen_robot) + "/kill";
  //topic_create_task =+ test.c_str();//"1";

//  ROS_INFO_STREAM("TOPIC: " << topic_kill_robot );
  _client2 = _nh.serviceClient<std_srvs::Empty>(topic_kill_robot);

  std_srvs::Empty srv;
  _client2.call(srv);
}

void Display_GUI::createNewTask()
{
  std::string topic_create_task = "/createNewTaskServer_ws_" + std::to_string(_chosen_machine);
  //topic_create_task =+ test.c_str();//"1";

  ROS_INFO_STREAM("TOPIC: " << topic_create_task );
  _client1 = _nh.serviceClient<std_srvs::Empty>(topic_create_task);

  std_srvs::Empty srv;
  _client1.call(srv);
}

void Display_GUI::callRos(void)
{
  if (!ros::ok())
  {
    QApplication::quit();
  }
  ros::spinOnce();
}

void Display_GUI::refreshInformation(const ros::TimerEvent& event)
{
  // *** Initialization: set all variables 0 ***
//  _chosen_machine = 0;
//  _chosen_robot = 0;
  _machine_total_tasks = 0;
  _robot_total_tasks = 0;

  Display_GUI::fillComboBoxRobots();
  Display_GUI::fillComboBoxMachines();
  Display_GUI::printInformation();

  // display general information
  _guiui->lcdNumberTotalTasks->display(_total_tasks);
  _guiui->lcdNumberOpenTasks->display(_open_tasks);
  _guiui->lcdNumberWorkingTasks->display(_working_tasks);
  _guiui->lcdNumberFinishedTasks->display(_finished_tasks);
  _guiui->lcdNumberFailedTasks->display(_failed_tasks);

  // value gives int from 1-9 according to machine-ID
  // _chosen_machine should be 0-8  (9 machines)!!
  //_chosen_machine = _guiui->spinBoxMachines->value();
  _chosen_machine = _guiui->comboBoxMachines->currentText().toInt(0, 10);

  int chosen_mach_index = _chosen_machine - 1;

  // counting total number of tasks for chosen machine
  for (int i = 0; i<4; i++)
  {
    _machine_total_tasks = _machine_total_tasks + _mach_tasks_status[chosen_mach_index][i];
  }

  //display machine information
  _guiui->lcdNumberMachineTotalTasks->display(_machine_total_tasks);
  _guiui->lcdNumberMachineOpenTasks->display(_mach_tasks_status[chosen_mach_index][0]);
  _guiui->lcdNumberMachineWorkingTasks->display(_mach_tasks_status[chosen_mach_index][1]);
  _guiui->lcdNumberMachineFinishedTasks->display(_mach_tasks_status[chosen_mach_index][2]);
  _guiui->lcdNumberMachineFailedTasks->display(_mach_tasks_status[chosen_mach_index][3]);

  // robot-ID between 0-5, value returns int from 0-5
  // _chosen_robot should be between 0-5 (6 robots)
  _chosen_robot = _guiui->comboBoxRobots->currentText().toInt(0, 10);

  // adding number of primary & secondary tasks of chosen robot
  _robot_total_tasks = _primary_robot_tasks[_chosen_robot] + _secondary_robot_tasks[_chosen_robot];

  // display robot information
  _guiui->lcdNumberRobotTotalTasks->display(_robot_total_tasks);
  _guiui->lcdNumberRobotPrimaryTasks->display(_primary_robot_tasks[_chosen_robot]);
  _guiui->lcdNumberRobotSecondaryTasks->display(_secondary_robot_tasks[_chosen_robot]);
}

bool Display_GUI::printInformation()
{
  // *** Initialization: set all variables 0 ***
  _total_tasks = 0;
  _open_tasks = 0;
  _working_tasks = 0;
  _finished_tasks = 0;
  _failed_tasks = 0;

  for (int i = 0; i < 6; i++)
  {
    _primary_robot_tasks [i] = 0;
    _secondary_robot_tasks [i] = 0;
  }

  for (int i = 0; i < 9; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      _mach_tasks_status [i][j] = 0;
    }
  }

//  std::cout << std::endl << "Taskhistory:" << std::endl << std::endl;
  for (unsigned int k = 0; k < h_tasklist.tasks.size(); k++)
  {
//    if (h_tasklist.tasks[k].task_status == 9)
//    {
//      ROS_INFO_STREAM("Initialization task");
//    }
//    ROS_INFO_STREAM("Task-ID: " << h_tasklist.tasks[k].tid << " Timestamp: "<< h_tasklist.tasks[k].header.stamp);
//    //ROS_INFO_STREAM("Task-Status: " << std::to_string(h_tasklist.tasks[k].task_status));
//    switch (h_tasklist.tasks[k].task_status)
//    {
//    case 0:
//      ROS_INFO_STREAM("Task-Status: open");
//      break;
//    case 1:
//      ROS_INFO_STREAM("Task-Status: working");
//      break;
//    case 2:
//      ROS_INFO_STREAM("Task-Status: finished");
//      break;
//    case 3:
//      ROS_INFO_STREAM("Task-Status: ERROR");
//      break;
//    default:
//      break;
//    }
//    ROS_INFO_STREAM("Primary Robot: " << h_tasklist.tasks[k].rid1 << " Secondary Robot: " << h_tasklist.tasks[k].rid2);
//    std::cout << std::endl;

    if (h_tasklist.tasks[k].taskStatus != 9)
    {
      switch (h_tasklist.tasks[k].tid)
      {
      case 1:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [0][0]++;
          break;
        case 1:
          _mach_tasks_status [0][1]++;
          break;
        case 2:
          _mach_tasks_status [0][2]++;
          break;
        case 3:
          _mach_tasks_status [0][3]++;
          break;
        }
        break;
      case 2:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [1][0]++;
          break;
        case 1:
          _mach_tasks_status [1][1]++;
          break;
        case 2:
          _mach_tasks_status [1][2]++;
          break;
        case 3:
          _mach_tasks_status [1][3]++;
          break;
        }
        break;
      case 3:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [2][0]++;
          break;
        case 1:
          _mach_tasks_status [2][1]++;
          break;
        case 2:
          _mach_tasks_status [2][2]++;
          break;
        case 3:
          _mach_tasks_status [2][3]++;
          break;
        }
        break;
      case 4:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [3][0]++;
          break;
        case 1:
          _mach_tasks_status [3][1]++;
          break;
        case 2:
          _mach_tasks_status [3][2]++;
          break;
        case 3:
          _mach_tasks_status [3][3]++;
          break;
        }
        break;
      case 5:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [4][0]++;
          break;
        case 1:
          _mach_tasks_status [4][1]++;
          break;
        case 2:
          _mach_tasks_status [4][2]++;
          break;
        case 3:
          _mach_tasks_status [4][3]++;
          break;
        }
        break;
      case 6:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [5][0]++;
          break;
        case 1:
          _mach_tasks_status [5][1]++;
          break;
        case 2:
          _mach_tasks_status [5][2]++;
          break;
        case 3:
          _mach_tasks_status [5][3]++;
          break;
        }
        break;
      case 7:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [6][0]++;
          break;
        case 1:
          _mach_tasks_status [6][1]++;
          break;
        case 2:
          _mach_tasks_status [6][2]++;
          break;
        case 3:
          _mach_tasks_status [6][3]++;
          break;
        }
        break;
      case 8:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [7][0]++;
          break;
        case 1:
          _mach_tasks_status [7][1]++;
          break;
        case 2:
          _mach_tasks_status [7][2]++;
          break;
        case 3:
          _mach_tasks_status [7][3]++;
          break;
        }
        break;
      case 9:
        switch (h_tasklist.tasks[k].taskStatus)
        {
        case 0:
          _mach_tasks_status [8][0]++;
          break;
        case 1:
          _mach_tasks_status [8][1]++;
          break;
        case 2:
          _mach_tasks_status [8][2]++;
          break;
        case 3:
          _mach_tasks_status [8][3]++;
          break;
        }
        break;
      default:
        break;
      }

      switch (h_tasklist.tasks[k].rid1)
      {
      case 0:
        _primary_robot_tasks[0]++;
        break;
      case 1:
        _primary_robot_tasks[1]++;
        break;
      case 2:
        _primary_robot_tasks[2]++;
        break;
      case 3:
        _primary_robot_tasks[3]++;
        break;
      case 4:
        _primary_robot_tasks[4]++;
        break;
      case 5:
        _primary_robot_tasks[5]++;
        break;
      default:
        break;
      }

      switch (h_tasklist.tasks[k].rid2)
      {
      case 0:
        _secondary_robot_tasks[0]++;
        break;
      case 1:
        _secondary_robot_tasks[1]++;
        break;
      case 2:
        _secondary_robot_tasks[2]++;
        break;
      case 3:
        _secondary_robot_tasks[3]++;
        break;
      case 4:
        _secondary_robot_tasks[4]++;
        break;
      case 5:
        _secondary_robot_tasks[5]++;
        break;
      default:
        break;
      }

      switch (h_tasklist.tasks[k].taskStatus)
      {
      case 0:
        _open_tasks++;
        break;
      case 1:
        _working_tasks++;
        break;
      case 2:
        _finished_tasks++;
        break;
      case 3:
        _failed_tasks++;
        break;
      default:
        break;
      }
    }       //end-if
  }         //end-for

  for (int i = 0; i < 9; i++)
  {
    for (int j=0; j<4; j++)
    {
      _total_tasks = _total_tasks + _mach_tasks_status [i][j];
    }
  }       //counting the total number of tasks

//  std::cout << "Statistics:" << std::endl << std::endl;
//
//  std::cout << "Total number of tasks: " << _total_tasks << std::endl;
//  std::cout << "Total number of open tasks: " << _open_tasks << std::endl;
//  std::cout << "Total number of working tasks: " << _working_tasks << std::endl;
//  std::cout << "Total number of finished tasks: " << _finished_tasks << std::endl;
//  std::cout << "Total number of failed tasks: " << _failed_tasks << std:: endl << std::endl;
//
//  for (int i=0; i<9; i++)
//  {
//    std::cout << "Machine " << i+1 << ":" << std::endl;
//    std::cout << "open: " << _mach_tasks_status[i][0] << std::endl;
//    std::cout << "working: " << _mach_tasks_status[i][1] << std::endl;
//    std::cout << "finished: " << _mach_tasks_status[i][2] << std::endl;
//    std::cout << "failed: " << _mach_tasks_status[i][3] << std::endl;
//    std::cout << std::endl;
//  }
//
//  for (int j = 0; j < 6; j++)
//  {
//    std::cout << "Robot " << j << ":" << std::endl;
//    std::cout << "Primary tasks: " << _primary_robot_tasks[j] << " Secondary tasks: " << _secondary_robot_tasks[j] << std::endl;
//  }
//  std::cout << std::endl;

  return true;
}


