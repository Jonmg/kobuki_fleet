/*
 * youbot_snapshot.cpp
 *
 *  Created on: 19.06.2016
 *      Author: dominik
 */

#include "youbot_snapshot.h"

youbot_snapshot::youbot_snapshot() :
	_prvNh("~")
{
	// TODO Auto-generated constructor stub
	_prvNh.param<std::string>("laserTopic", _laserTopic, "/sick/scan");
	_prvNh.param<std::string>("mapFrame", _mapFrame, "/map");
	_prvNh.param<std::string>("baseFootprintFrame", _baseFootprintFrame, "/base_link");

	_laserscan = _nh.subscribe(_laserTopic, 10, &youbot_snapshot::laserCallback, this);
	_serviceInventory = _nh.serviceClient<kobuki_fleet_msgs::setInventoryData>("setInventoryData");

	std::cout << "Welcome to Snapshot!" << std::endl;
}

youbot_snapshot::~youbot_snapshot() {
	// TODO Auto-generated destructor stub
	_laserscan.shutdown();
	_serviceInventory.shutdown();
}

void youbot_snapshot::laserCallback(const sensor_msgs::LaserScan& laserScan)
{
	_laser = laserScan;
}

tf::Transform youbot_snapshot::getRobotPose()
{
	tf::StampedTransform transform;
	try{
	  _listener.lookupTransform(_mapFrame, _baseFootprintFrame,
	                           ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
	  ROS_ERROR("%s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	return transform;
}


void youbot_snapshot::main(){

	std::string station = "";
	int materialtype = 0;
	double station_height = 0;
	int election = 0;

	kobuki_fleet_msgs::setInventoryData InventoryData;

	std::cout << "\nWhat would you like to save?\n";
	std::cout << "[1] Pose + Station Height + Laserscan\n";
	std::cout << "[2] Laserscan\n";
	std::cout << "[3] Station Height\n";
	std::cout << "[4] Pose\n";
	std::cout << "[5] Source Pose + Material Type\n";
	std::cout << "[0] Shutdown\n";
	std::cin >> election;

	if (election == 1) {


		std::cout << "Pose + Station Height + Laserscan has been elected"
				<< std::endl;
		std::cout << "What's the name of the actual station?" << std::endl;
		std::cin >> station;
		std::cout << "What's the height of the actual station? [in meters]" << std::endl;
		std::cin >> station_height;

		ros::spinOnce();

		tf::Transform robotTransform;
		robotTransform = getRobotPose();
		geometry_msgs::Quaternion geomQuaternion;
		tf::quaternionTFToMsg(robotTransform.getRotation(), geomQuaternion);

		InventoryData.request.start.data = true;
		InventoryData.request.pose.position.x = robotTransform.getOrigin().x();
		InventoryData.request.pose.position.y = robotTransform.getOrigin().y();
		InventoryData.request.pose.position.z = robotTransform.getOrigin().z();
		InventoryData.request.pose.orientation = geomQuaternion;
		InventoryData.request.laserScan = _laser;
		InventoryData.request.actual_station.data = station;
		InventoryData.request.station_height.data = station_height;
		_serviceInventory.call(InventoryData);

		response(InventoryData);

	} else if (election == 2) {

		std::cout << "Saving Laserscan has been elected" << std::endl;
		std::cout << "What's the name of the actual station?" << std::endl;
		std::cin >> station;

		ros::spinOnce();

		InventoryData.request.start.data = true;
		InventoryData.request.laserScan = _laser;
		InventoryData.request.actual_station.data = station;
		_serviceInventory.call(InventoryData);

		response(InventoryData);

	} else if (election == 3) {

		std::cout << "Saving Station Height has been elected" << std::endl;
		std::cout << "What's the name of the actual station?" << std::endl;
		std::cin >> station;
		std::cout << "What's the height of the actual station? [in meters]" << std::endl;
		std::cin >> station_height;

		InventoryData.request.start.data = true;
		InventoryData.request.station_height.data = station_height;
		InventoryData.request.actual_station.data = station;
		_serviceInventory.call(InventoryData);

		response(InventoryData);

	} else if (election == 4) {

		std::cout << "Saving Pose has been elected" << std::endl;
		std::cout << "What's the name of the actual station?" << std::endl;
		std::cin >> station;

		tf::Transform robotTransform;
		robotTransform = getRobotPose();
		geometry_msgs::Quaternion geomQuaternion;
		tf::quaternionTFToMsg(robotTransform.getRotation(), geomQuaternion);

		InventoryData.request.start.data = true;
		InventoryData.request.pose.position.x = robotTransform.getOrigin().x();
		InventoryData.request.pose.position.y = robotTransform.getOrigin().y();
		InventoryData.request.pose.position.z = robotTransform.getOrigin().z();
		InventoryData.request.pose.orientation = geomQuaternion;
		InventoryData.request.actual_station.data = station;
		_serviceInventory.call(InventoryData);

		response(InventoryData);

	} else if (election == 5) {

		std::cout << "Saving Source Pose + Material Type has been elected" << std::endl;
		std::cout << "What's the name of the actual source?" << std::endl;
		std::cin >> station;
		std::cout << "What's the material type of the actual source?" << std::endl;
		std::cin >> materialtype;

		tf::Transform robotTransform;
		robotTransform = getRobotPose();
		geometry_msgs::Quaternion geomQuaternion;
		tf::quaternionTFToMsg(robotTransform.getRotation(), geomQuaternion);

		InventoryData.request.start.data = true;
		InventoryData.request.pose.position.x = robotTransform.getOrigin().x();
		InventoryData.request.pose.position.y = robotTransform.getOrigin().y();
		InventoryData.request.pose.position.z = robotTransform.getOrigin().z();
		InventoryData.request.pose.orientation = geomQuaternion;
		InventoryData.request.actual_station.data = station;
		InventoryData.request.material_type.data = materialtype;
		_serviceInventory.call(InventoryData);

		response(InventoryData);

	} else if (election == 0) {

		shutdown();

	} else {
		std::cout << "Nothing has been elected" << std::endl;
		std::cout << "\n----------";
		std::cout << "Restarting";
		std::cout << "----------\n" << std::endl;
	}

}

void youbot_snapshot::response(kobuki_fleet_msgs::setInventoryData InventoryData) {
	if (InventoryData.response.finished.data == true) {
		std::cout << "Data has been stored" << std::endl;
		std::cout << "\n----------";
		std::cout << "Restarting";
		std::cout << "----------\n" << std::endl;
	} else {
		std::cout << "Something went wrong" << std::endl;
		std::cout << "\n----------";
		std::cout << "Restarting";
		std::cout << "----------\n" << std::endl;
	}
}

void youbot_snapshot::shutdown() {
	std::cout << "Shutting down" << std::endl;
	ros::shutdown();
}

void youbot_snapshot::start(){

	ros::Rate r(30);

	while (ros::ok()) {
		main();
		ros::spinOnce();
		r.sleep();
	}
}

////handler to shutdown node
//void mySigintHandler(int sig) {
//
//	ros::shutdown();
//}



