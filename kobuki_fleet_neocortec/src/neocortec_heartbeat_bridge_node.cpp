/*
 * neocortec_heartbeat_bridge_node.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: amndan
 */

#include <ros/ros.h>
#include <kobuki_fleet_msgs/HeartBeat.h>
#include <neocortec/neocortecData.h>
#include <assert.h>

/**
 * @brief Calback function for neocortec heartbeat message from neocortec node.
 * If some neocortec data is received by this callback it checks if its an neocortec
 * heartbeat. If yes it deserializes the data and resends the heartbeat through wifi.
 * @param message container
 */
void calNCHeartbeat(const neocortec::neocortecDataConstPtr& msg);

ros::Subscriber g_sub_nc_data;
ros::Publisher g_pub_wifi_heartbeat;

std::string g_top_wifi_heartbeat;
std::string g_top_nc_data;

uint8_t g_neocortec_rssi_node_id;

/**
 * @brief heartbeat bridge node converts heartbeat msg from neocortec heartbeats to wifi heartbeats
 * @param argc
 * @param argv
 * @return returnvalue
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "neocortec_heartbeat_bridge_node");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle prv_nh = ros::NodeHandle("~");

  int temp_int;

  prv_nh.param<std::string>("top_wifi_heartbeat", g_top_wifi_heartbeat, "/HB");
  prv_nh.param<std::string>("top_nc_heartbeat", g_top_nc_data, "neocortec_00_received_data");
  prv_nh.param<int>("neocortec_rssi_node", temp_int, 4);
  g_neocortec_rssi_node_id = temp_int;

  g_pub_wifi_heartbeat = nh.advertise<kobuki_fleet_msgs::HeartBeat>(g_top_wifi_heartbeat, 10);

  g_sub_nc_data = nh.subscribe(g_top_nc_data, 10, calNCHeartbeat);

  ros::spin();

  return EXIT_SUCCESS;
}


void calNCHeartbeat(const neocortec::neocortecDataConstPtr& nc_msg)
{
  if(nc_msg->nodeId == g_neocortec_rssi_node_id)
  {
    ROS_INFO("heart beat bridge received data from id %d, this is rssi node --> jump message.", nc_msg->nodeId);
    return;
  }

  ROS_INFO("heatbeatbridge: got nc heartbeat msg");

  kobuki_fleet_msgs::HeartBeat wifi_msg;

  //wifi msg:
  //Header header
  //uint16 rid
  //uint8 rob_status
  //uint8 OK=0
  //uint8 DISCONNECTED=1
  //float32 x
  //float32 y
  //float32 yaw
  //bool nchb

  // nc msg:
  // |nc header--->           |payload max 19 byte--->                                     |
  // |nid1|nid2|age1|age2|port|rid1|rid2|stat|x1|x2|x3|x4|y1|y2|y3|y4|j1|j2|j3|j4|nch|-|-|-|

  // get size of byte array
  const unsigned int nc_data_size = nc_msg->data.size();

  // check size of data
  if(nc_data_size != 16)
  {
    ROS_ERROR("neocortec data size = 16 assertion failed");
    return;
  }

  // create array
  uint8_t nc_data[nc_data_size];

  // copy to c array
  for(unsigned int i = 0; i < nc_data_size; i++)
    nc_data[i] = nc_msg->data.at(i);

  // deserialize byte array
  wifi_msg.header.stamp = nc_msg->header.stamp;
  wifi_msg.header.frame_id = "map";
  wifi_msg.rid =  *( (uint16_t*) (nc_data) );

  /**
   * @bug Warning: for the nid in hb via nc the sending nc-node is used
   * --> better way would be to add nid to the message.
   * If not the message cannot be resent via nc by e.g. a node
   * without wifi connection
   */
  wifi_msg.nid = nc_msg->nodeId;

  wifi_msg.rob_status = *(nc_data + 2);
  wifi_msg.x = *( (float*) (nc_data + 3) );
  wifi_msg.y = *( (float*) (nc_data + 7) );
  wifi_msg.yaw = *( (float*) (nc_data + 11) );
  wifi_msg.nchb = (bool) ( *(nc_data + 15) );

  // publish message
  g_pub_wifi_heartbeat.publish(wifi_msg);
}




