
#include "model.h"

/**
 * @file model.cpp
 * @brief contains implementation of class Model
 */

Model::Model(const std_msgs::UInt16& robotId, ros::NodeHandle* nh):
    nh_(nh),
    robotId_(robotId),
    simulation_(0)
{

}

Model::~Model()
{

}

kobuki_fleet_msgs::Task const Model::curTask(void) const
{
//    ROS_INFO_STREAM("returned current task from model function called. CurrentTask.task: " << curTask_.task );
  return curTask_;
}

bool Model::setCurentTask(kobuki_fleet_msgs::Task curTask)
{
  curTask_ = curTask;
  return true;
}

void Model::setInventorySources(const kobuki_fleet_msgs::LocationIdentifier inventoyLocations)
{
  inventoyLocations_ = inventoyLocations;
}

bool Model::getInventorySources(kobuki_fleet_msgs::LocationIdentifier& inventoryLocations)
{
  inventoryLocations = inventoyLocations_;
  return true;
}

void Model::addSubTasks(const kobuki_fleet_msgs::SubTaskVector subTaskVector)
{
  _subTaskVector = subTaskVector;
}

bool Model::getSubTasks(kobuki_fleet_msgs::SubTaskVector& subTaskVector)
{
  if(!_subTaskVector.subtasks.size())
  {
    return false;
  }
  subTaskVector = _subTaskVector;
  return true;
}

void Model::setActualSubTask(kobuki_fleet_msgs::SubTask task)
{
  _actualSubTask = task;
}

void Model::getActualSubTask(kobuki_fleet_msgs::SubTask& task)
{
  task = _actualSubTask;
}

