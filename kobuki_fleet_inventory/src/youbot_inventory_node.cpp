/*
 * youbot_inventory_node.cpp
 *
 *  Created on: 11.06.2016
 *      Author: dominik
 */

#include "youbot_inventory.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "kobuki_fleet_inventory");
	youbot_inventory inventory;
	inventory.start();
}



