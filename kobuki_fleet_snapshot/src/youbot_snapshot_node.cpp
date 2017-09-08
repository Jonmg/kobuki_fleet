/*
 * youbot_snapshot_node.cpp
 *
 *  Created on: 19.06.2016
 *      Author: dominik
 */

#include "youbot_snapshot.h"

int main(int argc, char **argv)
{
//	ros::init(argc, argv, "youbot_snapshot", ros::init_options::NoSigintHandler);
	ros::init(argc, argv, "youbot_snapshot");
	youbot_snapshot snapshot;
	snapshot.start();

}


