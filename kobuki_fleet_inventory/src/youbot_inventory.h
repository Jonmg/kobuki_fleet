/*
 * youbot_inventory.h
 *
 *  Created on: 11.06.2016
 *      Author: dominik
 */

#ifndef YOUBOT_INVENTORY_H_
#define YOUBOT_INVENTORY_H_

#include <ros/ros.h>
#include <ros/service.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <cstring>
#include <string>
#include <sys/stat.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <dirent.h>

#include <opencv2/opencv.hpp>


#include <kobuki_fleet_msgs/getInventoryData.h>
#include <kobuki_fleet_msgs/setInventoryData.h>
#include "kobuki_fleet_msgs/getAllInventoryData.h"
#include <kobuki_fleet_msgs/LocationIdentifier.h>


class youbot_inventory {

public:
  youbot_inventory();
  virtual ~youbot_inventory();
  bool getInventoryData(kobuki_fleet_msgs::getInventoryData::Request& req, kobuki_fleet_msgs::getInventoryData::Response& res );
  bool setInventoryData(kobuki_fleet_msgs::setInventoryData::Request& req, kobuki_fleet_msgs::setInventoryData::Response& res );
  bool getAllInventoryData(kobuki_fleet_msgs::getAllInventoryData::Request& req, kobuki_fleet_msgs::getAllInventoryData::Response& res);
  void wsTfPrefixer();
  void tfbroadcaster(std::vector<double> quaternion,std::vector<double> pose,std::string frame_id,std::string frame_cluster);
  void start();

private:
  ros::NodeHandle _nh;
  ros::NodeHandle _prvNh;
  ros::ServiceServer _serviceGetInventory;
  ros::ServiceServer _serviceSetInventoryData;
  ros::ServiceServer _serviceGetAllInventory;

  tf::TransformBroadcaster _br;
//  tf::Transform _transform;

  std::string _F20;
  std::string _S40;
  std::string _M20;
  std::string _M30;
  std::string _R20;
  std::string _default;

  std::vector<std::string> _fileList;

  std::string _mapFrame;
  std::string _clusterPrefix;
  std::string _packageName;

  bool _TFclusterOnOff;
  bool _readOnly;
  int _listIter;

  double _xTFClusterOffset;
  double _yTFClusterOffset;
  double _rosRate;
};

#endif /* YOUBOT_INVENTORY_H_ */
