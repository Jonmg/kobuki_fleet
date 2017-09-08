/*
 * youbot_snapshot.h
 *
 *  Created on: 19.06.2016
 *      Author: dominik
 */

#ifndef YOUBOT_SNAPSHOT_H_
#define YOUBOT_SNAPSHOT_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <sensor_msgs/LaserScan.h>

#include <kobuki_fleet_msgs/setInventoryData.h>

#include <signal.h>

////handler to shutdown node
//void mySigintHandler(int sig);

class youbot_snapshot {

public:
	youbot_snapshot();
	virtual ~youbot_snapshot();
	void laserCallback(const sensor_msgs::LaserScan& laserScan);
	void main();
	void response(kobuki_fleet_msgs::setInventoryData InventoryData);
	void shutdown();
	void start();

private:
	tf::Transform getRobotPose();
	ros::NodeHandle _nh;
	ros::NodeHandle _prvNh;
	ros::Subscriber _laserscan;
	ros::ServiceClient _serviceInventory;
	tf::TransformListener _listener;

	sensor_msgs::LaserScan _laser;

	std::string _laserTopic;
	std::string _mapFrame;
	std::string _baseFootprintFrame;
};

#endif /* YOUBOT_SNAPSHOT_H_ */
