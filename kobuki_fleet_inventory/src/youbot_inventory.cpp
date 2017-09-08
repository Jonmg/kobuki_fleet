/*
 * youbot_inventory.cpp
 *
 *  Created on: 11.06.2016
 *      Author: dominik
 */

#include "youbot_inventory.h"

youbot_inventory::youbot_inventory():
_prvNh("~")
{
  // TODO Auto-generated constructor stub
  _prvNh.param<double>("xTFOffset", _xTFClusterOffset, -0.5);
  _prvNh.param<double>("yTFOffset", _yTFClusterOffset, 0.2);
  _prvNh.param<std::string>("mapFrame", _mapFrame, "/map");
  _prvNh.param<std::string>("clusterPrefix", _clusterPrefix, "C_");
  _prvNh.param<std::string>("packageName", _packageName, "youbot_inventory");
  _prvNh.param<bool>("TFclusterOnOff", _TFclusterOnOff, false);
  _prvNh.param<double>("rosRate", _rosRate, 30);

  _serviceGetInventory = _nh.advertiseService("getInventoryData", &youbot_inventory::getInventoryData, this);
  _serviceGetAllInventory = _nh.advertiseService("getAllInventoryData", &youbot_inventory::getAllInventoryData, this);

  _serviceSetInventoryData = _nh.advertiseService("setInventoryData", &youbot_inventory::setInventoryData, this);

  _F20 = "F20_20_B";
  _S40 = "S40_40_B";
  _M20 = "M20";
  _M30 = "M30";
  _R20 = "R20";
  _default = "default";
  _readOnly = false;
  _listIter = 0;

  //Check if workstation folder exists and create if needed
  std::stringstream dirPathSS;
  dirPathSS << ros::package::getPath(_packageName)<< "/Workstations";
  std::string dirPath = dirPathSS.str();

  struct stat sb;
  if(stat(dirPath.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode)){
    ROS_DEBUG_STREAM("Could open " + dirPath);
  }else{
    ROS_ERROR_STREAM("Couldn't open " + dirPath);
    mkdir(dirPath.c_str(),0777);
    ROS_INFO_STREAM("Created new Folder");
  }
  //~Check if workstation folder exists and create if needed

  std::cout << "Inventory ready!" << std::endl;

  _fileList.clear();

}

youbot_inventory::~youbot_inventory() {
  // TODO Auto-generated destructor stub
}

inline bool exists_test(const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

inline bool exists_directory(const std::string& path) {
  struct stat sb;
  return (stat(path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode));
}

bool youbot_inventory::getAllInventoryData(kobuki_fleet_msgs::getAllInventoryData::Request& req, kobuki_fleet_msgs::getAllInventoryData::Response& res)
{
  std::vector<double> orientation;
  std::vector<double> quaternion;
  geometry_msgs::Pose pose;
  std::string locationName;
  kobuki_fleet_msgs::LocationIdentifier location;

  if (!_fileList.size())
  {
    ROS_WARN("Inventory folder is empty");
    res.finished.data = false;
    return false;
  }

  for(int i=0; i<_fileList.size();i++)
  {
    int materialType;
    std::stringstream InventoryPathSS;
    InventoryPathSS << ros::package::getPath(_packageName) << "/Workstations/" << _fileList.at(i);
    std::string InventoryPath = InventoryPathSS.str();

    //Check if station and object are appended
    if (exists_test(InventoryPath)) {

      //Open and read file
      cv::FileStorage fs_read(InventoryPath.c_str(), cv::FileStorage::READ);

      if(fs_read.isOpened())
        ROS_DEBUG("Dir opened!");
      else{
        ROS_ERROR("DIr didn't open!");
        res.finished.data = false;
        return false;
      }

      fs_read["Workstation"] >> locationName;
      fs_read["Pose"] >> orientation;
      fs_read["Quaternion"] >> quaternion;
      fs_read["Material Type"] >> materialType;
      //Close file
      fs_read.release();

      pose.position.x = orientation[0];
      pose.position.y = orientation[1];
      pose.position.z = orientation[2];
      pose.orientation.x = quaternion[0];
      pose.orientation.y = quaternion[1];
      pose.orientation.z = quaternion[2];
      pose.orientation.w = quaternion[3];

      if (locationName.substr (0,2) == "SC")
          location.type.data = 1;
      else if (locationName.substr (0,2) == "MA")
        location.type.data = 2;
      location.instance_id.data = std::atoi(locationName.substr (2,1).c_str());
      location.description.data = locationName;
      location.materialType.data = materialType;
      location.pose = pose;

      res.locations.push_back(location);

    } else
    {
      ROS_WARN_STREAM("Inventory: " + _fileList.at(i) + " doesn't exist, review your call");
    }
  }

  ROS_INFO("Returning data from inventory");
  res.finished.data = true;
  return true;

}

bool youbot_inventory::getInventoryData(
		kobuki_fleet_msgs::getInventoryData::Request& req,
		kobuki_fleet_msgs::getInventoryData::Response& res) {

  std::vector<double> ranges;
  std::vector<double> quaternion;
  std::vector<double> pose;
  std::vector<int> cameraData_F20;
  std::vector<int> cameraData_S40;
  std::vector<int> cameraData_M20;
  std::vector<int> cameraData_M30;
  std::vector<int> cameraData_R20;
  std::vector<int> cameraData_default;

  cameraData_F20.resize(5);
  cameraData_S40.resize(5);
  cameraData_M20.resize(5);
  cameraData_M30.resize(5);
  cameraData_R20.resize(5);
  cameraData_default.resize(5);

  //Check if station and object are appended
  if(req.actual_station.data.size() == 0){
    ROS_WARN_STREAM("Inventory: " + req.actual_station.data + ".yml is empty, review your call");

    res.finished.data = false;
    return false;
  }

  std::stringstream InventoryPathSS;
  InventoryPathSS << ros::package::getPath(_packageName) << "/Workstations/" << req.actual_station.data << ".yml";
  std::string InventoryPath = InventoryPathSS.str();

  //Check if station and object are appended
  if (exists_test(InventoryPath)) {

    ROS_INFO_STREAM("Inventory: " + req.actual_station.data + ".yml exists, reading data");

    //Open and read file
    cv::FileStorage fs_read(InventoryPath.c_str(), cv::FileStorage::READ);

    if(fs_read.isOpened())
      ROS_DEBUG_STREAM("Dir opened!");
    else{
      ROS_ERROR_STREAM("DIr didn't open!");
      res.finished.data = false;
      return false;
    }

    fs_read["Station Height"] >> res.station_height.data;
    fs_read["Material Type"] >> res.material_type.data;

    if (req.object.data == _F20) {
      fs_read[_F20] >> cameraData_F20;
      res.brightness.data = cameraData_F20[0];
      res.filter.data = cameraData_F20[1];
      res.filter_size.data = cameraData_F20[2];
      res.threshold_1.data = cameraData_F20[3];
      res.threshold_2.data = cameraData_F20[4];
    } else if (req.object.data == _S40 ) {
      fs_read[_S40] >> cameraData_S40;
      res.brightness.data = cameraData_S40[0];
      res.filter.data = cameraData_S40[1];
      res.filter_size.data = cameraData_S40[2];
      res.threshold_1.data = cameraData_S40[3];
      res.threshold_2.data = cameraData_S40[4];
    } else if (req.object.data == _M20) {
      fs_read[_M20] >> cameraData_M20;
      res.brightness.data = cameraData_M20[0];
      res.filter.data = cameraData_M20[1];
      res.filter_size.data = cameraData_M20[2];
      res.threshold_1.data = cameraData_M20[3];
      res.threshold_2.data = cameraData_M20[4];
    } else if (req.object.data == _M30) {
      fs_read[_M30] >> cameraData_M30;
      res.brightness.data = cameraData_M30[0];
      res.filter.data = cameraData_M30[1];
      res.filter_size.data = cameraData_M30[2];
      res.threshold_1.data = cameraData_M30[3];
      cameraData_M30[4] = res.threshold_2.data;
    } else if (req.object.data == _R20) {
      fs_read[_R20] >> cameraData_R20;
      res.brightness.data = cameraData_R20[0];
      res.filter.data = cameraData_R20[1];
      res.filter_size.data = cameraData_R20[2];
      res.threshold_1.data = cameraData_R20[3];
      res.threshold_2.data = cameraData_R20[4];
    } else if ((req.object.data == _default) || req.object.data == "") {
      fs_read[_default] >> cameraData_default;
      res.brightness.data = cameraData_default[0];
      res.filter.data = cameraData_default[1];
      res.filter_size.data = cameraData_default[2];
      res.threshold_1.data = cameraData_default[3];
      res.threshold_2.data = cameraData_default[4];
    }

    //Laser
    fs_read["Frame_id"] >> res.laserScan.header.frame_id;
    fs_read["Angle_min"] >> res.laserScan.angle_min;
    fs_read["Angle_max"] >> res.laserScan.angle_max;
    fs_read["Range_min"] >> res.laserScan.range_min;
    fs_read["Range_max"] >> res.laserScan.range_max;
    fs_read["Angle_increment"] >> res.laserScan.angle_increment;
    fs_read["Ranges"] >> ranges;
    //Pose and Quaternion
    fs_read["Pose"] >> pose;
    fs_read["Quaternion"] >> quaternion;
    //Close file
    fs_read.release();

    res.pose.position.x = pose[0];
    res.pose.position.y = pose[1];
    res.pose.position.z = pose[2];

    res.pose.orientation.x = quaternion[0];
    res.pose.orientation.y = quaternion[1];
    res.pose.orientation.z = quaternion[2];
    res.pose.orientation.w = quaternion[3];

    res.laserScan.ranges.insert(res.laserScan.ranges.begin(),ranges.begin(),ranges.end()); //insert(req.laserScan.ranges.end(),ranges.begin(),ranges.end());

    ROS_INFO_STREAM("sending data");

    res.finished.data = true;
    return true;

  } else {

    ROS_WARN_STREAM("Inventory: " + req.actual_station.data + ".yml dosn't exist, review your call");

    res.finished.data = false;
    return false;
  }

  res.finished.data = false;
  return false;
}

bool youbot_inventory::setInventoryData(kobuki_fleet_msgs::setInventoryData::Request& req,
		kobuki_fleet_msgs::setInventoryData::Response& res) {

  //Check if station and object are appended
  if(req.actual_station.data.size() == 0){
    ROS_WARN_STREAM("Inventory: " + req.actual_station.data + ".yml dosn't exist, review your call");

    res.finished.data = false;
    return false;
  }

  std::vector<double> ranges;
  std::vector<double> quaternion;
  std::vector<double> pose;
  std::vector<int> cameraData_F20;
  std::vector<int> cameraData_S40;
  std::vector<int> cameraData_M20;
  std::vector<int> cameraData_M30;
  std::vector<int> cameraData_R20;
  std::vector<int> cameraData_default;

  cameraData_F20.resize(5);
  cameraData_S40.resize(5);
  cameraData_M20.resize(5);
  cameraData_M30.resize(5);
  cameraData_R20.resize(5);
  cameraData_default.resize(5);

  sensor_msgs::LaserScan laserScan;
  std::string frame_id = "";
  std::string ws = "";
  double stationh = 0.0;
  double angle_min = 0.0;
  double angle_max = 0.0;
  double range_min = 0.0;
  double range_max = 0.0;
  double angle_increment = 0.0;
  int materialType;
  int brightness = 0;
  int filter = 0;
  int filter_size = 0;
  int threshold1 = 0;
  int threshold2 = 0;

  std::stringstream InventoryPathSS;
  InventoryPathSS << ros::package::getPath(_packageName) << "/Workstations/" << req.actual_station.data << ".yml";
  std::string InventoryPath = InventoryPathSS.str();

  //Check for file
  //Create new file if needed
  if (exists_test(InventoryPath)) {

    ROS_WARN_STREAM("Inventory: " + req.actual_station.data + ".yml already exists, rewrite only");

    //Open and read file
    cv::FileStorage fs_read(InventoryPath.c_str(), cv::FileStorage::READ);

    if(fs_read.isOpened())
      ROS_DEBUG_STREAM("Dir opened!");
    else{
      ROS_ERROR_STREAM("DIr didn't open!");
      res.finished.data = false;
      return false;
    }

    fs_read["Workstation"] >> ws;
    fs_read["Station Height"] >> stationh;
    fs_read["Material Type"] >> materialType;
    //Objects
    fs_read[_F20] >> cameraData_F20;
    fs_read[_S40] >> cameraData_S40;
    fs_read[_M20] >> cameraData_M20;
    fs_read[_M30] >> cameraData_M30;
    fs_read[_R20] >> cameraData_R20;
    fs_read[_default] >> cameraData_default;
    //Laser
    fs_read["Frame_id"] >> frame_id;
    fs_read["Angle_min"] >> angle_min;
    fs_read["Angle_max"] >>angle_max;
    fs_read["Range_min"] >> range_min;
    fs_read["Range_max"] >>range_max;
    fs_read["Angle_increment"] >> angle_increment;
    fs_read["Ranges"] >> ranges;
    //Pose and Quaternion
    fs_read["Pose"] >> pose;
    fs_read["Quaternion"] >> quaternion;
    //Close file
    fs_read.release();

    //Rewrite Data if necessary
    if (req.actual_station.data.size() != 0)
      ws = req.actual_station.data;
    if (ceil(req.station_height.data) > 0.0)
      stationh = req.station_height.data;

    //Process laserscan
    if(req.laserScan.header.frame_id != ""){
      frame_id = req.laserScan.header.frame_id;
      angle_min = req.laserScan.angle_min;
      angle_max = req.laserScan.angle_max;
      range_min = req.laserScan.range_min;
      range_max = req.laserScan.range_max;
      angle_increment = req.laserScan.angle_increment;
      ranges.clear();
      ranges.insert(ranges.begin(),req.laserScan.ranges.begin(),req.laserScan.ranges.end());
    }

    if(req.pose.position.x != 0 || req.pose.position.y != 0 || req.pose.position.z != 0){
      pose.clear();
      pose.push_back(req.pose.position.x);
      pose.push_back(req.pose.position.y);
      pose.push_back(req.pose.position.z);
    }

    if(req.pose.orientation.x != 0 || req.pose.orientation.y != 0 || req.pose.orientation.z != 0 || req.pose.orientation.w != 0){
      quaternion.clear();
      quaternion.push_back(req.pose.orientation.x);
      quaternion.push_back(req.pose.orientation.y);
      quaternion.push_back(req.pose.orientation.z);
      quaternion.push_back(req.pose.orientation.w);
    }

    //Process objects
    if (req.object.data == _F20) {
      cameraData_F20[0] = req.brightness.data;
      cameraData_F20[1] = req.filter.data;
      cameraData_F20[2] = req.filter_size.data;
      cameraData_F20[3] = req.threshold_1.data;
      cameraData_F20[4] = req.threshold_2.data;
    } else if (req.object.data == _S40 ) {
      cameraData_S40[0] = req.brightness.data;
      cameraData_S40[1] = req.filter.data;
      cameraData_S40[2] = req.filter_size.data;
      cameraData_S40[3] = req.threshold_1.data;
      cameraData_S40[4] = req.threshold_2.data;
    } else if (req.object.data == _M20) {
      cameraData_M20[0] = req.brightness.data;
      cameraData_M20[1] = req.filter.data;
      cameraData_M20[2] = req.filter_size.data;
      cameraData_M20[3] = req.threshold_1.data;
      cameraData_M20[4] = req.threshold_2.data;
    } else if (req.object.data == _M30) {
      cameraData_M30[0] = req.brightness.data;
      cameraData_M30[1] = req.filter.data;
      cameraData_M30[2] = req.filter_size.data;
      cameraData_M30[3] = req.threshold_1.data;
      cameraData_M30[4] = req.threshold_2.data;
    } else if (req.object.data == _R20) {
      cameraData_R20[0] = req.brightness.data;
      cameraData_R20[1] = req.filter.data;
      cameraData_R20[2] = req.filter_size.data;
      cameraData_R20[3] = req.threshold_1.data;
      cameraData_R20[4] = req.threshold_2.data;
    }

  //Create new File
  } else {

    ROS_INFO_STREAM("Inventory: " + req.actual_station.data + ".yml doesn't exist, file will be created");

    ws = req.actual_station.data;
    stationh = req.station_height.data;
    materialType = req.material_type.data;

    //Process laserscan
    if(req.laserScan.header.frame_id != ""){
      frame_id = req.laserScan.header.frame_id;
      angle_min = req.laserScan.angle_min;
      angle_max = req.laserScan.angle_max;
      range_min = req.laserScan.range_min;
      range_max = req.laserScan.range_max;
      angle_increment = req.laserScan.angle_increment;
      ranges.insert(ranges.begin(),req.laserScan.ranges.begin(),req.laserScan.ranges.end());
    }

    pose.push_back(req.pose.position.x);
    pose.push_back(req.pose.position.y);
    pose.push_back(req.pose.position.z);

    quaternion.push_back(req.pose.orientation.x);
    quaternion.push_back(req.pose.orientation.y);
    quaternion.push_back(req.pose.orientation.z);
    quaternion.push_back(req.pose.orientation.w);

    //Process objects
    if (req.object.data == _F20) {
      cameraData_F20[0] = req.brightness.data;
      cameraData_F20[1] = req.filter.data;
      cameraData_F20[2] = req.filter_size.data;
      cameraData_F20[3] = req.threshold_1.data;
      cameraData_F20[4] = req.threshold_2.data;
    } else if (req.object.data == _S40 ) {
      cameraData_S40[0] = req.brightness.data;
      cameraData_S40[1] = req.filter.data;
      cameraData_S40[2] = req.filter_size.data;
      cameraData_S40[3] = req.threshold_1.data;
      cameraData_S40[4] = req.threshold_2.data;
    } else if (req.object.data == _M20) {
      cameraData_M20[0] = req.brightness.data;
      cameraData_M20[1] = req.filter.data;
      cameraData_M20[2] = req.filter_size.data;
      cameraData_M20[3] = req.threshold_1.data;
      cameraData_M20[4] = req.threshold_2.data;
    } else if (req.object.data == _M30) {
      cameraData_M30[0] = req.brightness.data;
      cameraData_M30[1] = req.filter.data;
      cameraData_M30[2] = req.filter_size.data;
      cameraData_M30[3] = req.threshold_1.data;
      cameraData_M30[4] = req.threshold_2.data;
    } else if (req.object.data == _R20) {
      cameraData_R20[0] = req.brightness.data;
      cameraData_R20[1] = req.filter.data;
      cameraData_R20[2] = req.filter_size.data;
      cameraData_R20[3] = req.threshold_1.data;
      cameraData_R20[4] = req.threshold_2.data;
    }

    cameraData_default[0] = 50;
    cameraData_default[1] = 1;
    cameraData_default[2] = 3;
    cameraData_default[3] = 30;
    cameraData_default[4] = 190;


  }

  //Write data
  cv::FileStorage fs_write(InventoryPath.c_str(), cv::FileStorage::WRITE);

  if(fs_write.isOpened())
    ROS_DEBUG_STREAM("Dir opened!");
  else{
    ROS_ERROR_STREAM("DIr didn't open!");
    res.finished.data = false;
    return false;
  }


  fs_write << "Workstation" << ws;
  fs_write << "Station Height" << stationh;
  fs_write << "Material Type" << materialType;
  //Objects
  fs_write << _F20 << cameraData_F20;
  fs_write << _S40 << cameraData_S40;
  fs_write << _M20 << cameraData_M20;
  fs_write << _M30 << cameraData_M30;
  fs_write << _R20 << cameraData_R20;
  fs_write << _default << cameraData_default;
  //Laser
  //ToDo: Layer
//  fs_write << "LaserScan" << "[";
  fs_write << "Frame_id" << frame_id;
  fs_write << "Angle_min" << angle_min;
  fs_write << "Angle_max" << angle_max;
  fs_write << "Range_min" << range_min;
  fs_write << "Range_max" << range_max;
  fs_write << "Angle_increment" << angle_increment;
  fs_write << "Ranges" << ranges;
//  fs_write << "]";
  fs_write << "Pose" << pose;
  fs_write << "Quaternion" << quaternion;
  //Close file
  fs_write.release();

  ROS_INFO_STREAM("Inventory: data has been stored");

  res.finished.data = true;
  return true;

}

void youbot_inventory::wsTfPrefixer() {
  DIR *dirHandle;
  struct dirent * dirEntry;
  unsigned char isFile = 0x8;
  std::string fullFileName;
  std::string FileName;
  int position;

  if (!_readOnly) {
    std::stringstream dirPathSS;
    dirPathSS << ros::package::getPath(_packageName)
        << "/Workstations";
    std::string dirPath = dirPathSS.str();

    //Open Folder with Wokrstations
    dirHandle = opendir(dirPath.c_str()); /* oeffne aktuelles Verzeichnis */
    if (dirHandle) {
      //Ignore ".." and "."
      while (0 != (dirEntry = readdir(dirHandle))) {
        if (!strcmp(dirEntry->d_name, "..")|| !strcmp(dirEntry->d_name, ".")) {
          continue;
        }
//        ROS_INFO_STREAM(fileList.size());
//        ROS_INFO_STREAM(dirEntry->d_name);

        _fileList.push_back(dirEntry->d_name);

      }
      closedir(dirHandle);

      _readOnly = true;

    } else
      ROS_ERROR_STREAM("Couldnt open " + dirPath);
  } else {

    std::vector<double> quaternion;
    std::vector<double> pose;
    std::string fullFileName;
    std::string frame_id;
    std::string frame_cluster;
    int position;

    // check if there are any files
    if (_fileList.size() == 0) {
      _readOnly = false;
      return;
    }

    //Create frame id's for tf
    fullFileName = _fileList[_listIter];
    position = fullFileName.find('.');
    frame_id = fullFileName.substr(0, position);
    frame_cluster = _clusterPrefix;
    frame_cluster.append(frame_id);

    //Get invnetory data
    std::stringstream InventoryPathSS;
    InventoryPathSS << ros::package::getPath(_packageName)
        << "/Workstations/" << frame_id << ".yml";
    std::string InventoryPath = InventoryPathSS.str();

      //Open and read file
    cv::FileStorage fs_read(InventoryPath.c_str(), cv::FileStorage::READ);
    //Pose and Quaternion
    fs_read["Pose"] >> pose;
    fs_read["Quaternion"] >> quaternion;
    //Close file
    fs_read.release();

//      std::cout << "Path to File : " << InventoryPathSS << std::endl;
//      std::cout << "Found a File : " << fullFileName << std::endl;
//      std::cout << "Frame ID : " << frame_id << std::endl;
//      std::cout << "Frame Cluster ID : " << frame_cluster << std::endl;

    //Publish tf + data
    tfbroadcaster(quaternion, pose, frame_id, frame_cluster);

    if (_listIter == (_fileList.size()-1)) {
      _listIter = 0;
      _readOnly = false;
      _fileList.clear();
    } else {
      _listIter++;
    }

  }
}

void youbot_inventory::tfbroadcaster(std::vector<double> quaternion,std::vector<double> pose, std::string frame_id, std::string frame_cluster){
  //Create Workstation TF
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(pose[0], pose[1], pose[2]));
  tf::Quaternion q;
  q.setX(quaternion[0]);
  q.setY(quaternion[1]);
  q.setZ(quaternion[2]);
  q.setW(quaternion[3]);
  transform.setRotation(q);
  _br.sendTransform(tf::StampedTransform(transform, ros::Time::now()+ros::Duration(2.0), _mapFrame,frame_id)); // future date transformation
  //Create Cluster TF
  if (_TFclusterOnOff) {
    transform.setOrigin(tf::Vector3(_xTFClusterOffset, _yTFClusterOffset, 0.0));
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    _br.sendTransform(tf::StampedTransform(transform, ros::Time::now()+ros::Duration(2.0), frame_id,frame_cluster));  // future date transformation
  }
}

void youbot_inventory::start() {

  ros::Rate r(_rosRate);

  while (ros::ok()) {
    wsTfPrefixer();
    ros::spinOnce();
    r.sleep();
  }
}
