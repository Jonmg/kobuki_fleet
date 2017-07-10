/*
 * neocortec_rssi_bridge_node.cpp
 *
 *  Created on: Sep 2, 2016
 *      Author: amndan
 */

#include <ros/ros.h>
#include <neocortec/neocortecData.h>
#include <assert.h>
#include <kobuki_fleet_msgs/HeartBeatList.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>

#define RSSI_THRESH -1000 // get all measurements --> let pf decide which one to use...

/**
 * @brief calback function for heartbeat message from all robots
 * @param message container
 */
void calHeartbeatList(const kobuki_fleet_msgs::HeartBeatListConstPtr& msg);

void calNCData(const neocortec::neocortecDataConstPtr& nc_msg);

ros::Subscriber g_sub_nc_data;
ros::Subscriber g_sub_heart_beat_list;
ros::Publisher g_pub_poses;

std::string g_top_poses;
std::string g_top_nc_data;
std::string g_top_heart_beat_list;

uint8_t g_neocortec_rssi_node_id;

kobuki_fleet_msgs::HeartBeatList g_heart_beat_list;


int main(int argc, char** argv)
{
  ros::init(argc, argv, "neocortec_rssi_bridge_node");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle prv_nh = ros::NodeHandle("~");

  int temp_int;

  prv_nh.param<std::string>("top_nc_data", g_top_nc_data, "neocortec_00_received_data");
  prv_nh.param<std::string>("top_heart_beat_list", g_top_heart_beat_list, "heart_beat");
  prv_nh.param<std::string>("top_poses", g_top_poses, "rssi_poses");
  prv_nh.param<int>("neocortec_rssi_node_id", temp_int, 3);
  g_neocortec_rssi_node_id = temp_int;

  g_pub_poses = nh.advertise<geometry_msgs::PoseArray>(g_top_poses, 1);

  g_sub_nc_data = nh.subscribe(g_top_nc_data, 10, calNCData);
  g_sub_heart_beat_list = nh.subscribe(g_top_heart_beat_list, 10, calHeartbeatList);

  ros::spin();

  return EXIT_SUCCESS;
}


void calNCData(const neocortec::neocortecDataConstPtr& nc_msg)
{ 
  if(nc_msg->nodeId != g_neocortec_rssi_node_id)
  {
    ROS_INFO_STREAM("nc message received --> wrong id; should be " << (int)  g_neocortec_rssi_node_id <<  " --> id = " << (int)  nc_msg->nodeId << " --> will jump package");
    return;
  }

  ROS_INFO("rssi bridge: got nc rssi msg");

  if( (nc_msg->data.size() - 1) % 3 )
  {
    ROS_WARN_STREAM("neocortec rssi data is of wrong format --> will jump that package...");
    return;
  }

  uint8_t byteArray[ nc_msg->data.size() - 1 ];
  std::vector<uint16_t> ids;
  std::vector<uint8_t> rssi_values;

  for(unsigned int i = 0; i < nc_msg->data.size() - 1; i++)
  {
    byteArray[i] = nc_msg->data.at(i+1);
  }

  for(unsigned int i = 0; i < (nc_msg->data.size() - 1) / 3; i++)
  {
    // neocortec store in little endian --> we need big endian:
    uint8_t p_bytes[2];
    uint16_t* p_id;

    p_id = (uint16_t*) p_bytes;

    // swap endianes
    memcpy(p_bytes, byteArray + 3 * i + 1, sizeof(byteArray[0]));
    memcpy(p_bytes+1, byteArray + 3 * i, sizeof(byteArray[0]));

    ids.push_back( *p_id );
    rssi_values.push_back( *( (uint8_t*) (byteArray + 2 + 3 * i) ) );
  }

  geometry_msgs::PoseArray poses;
  poses.header.stamp = ros::Time::now();
  poses.header.frame_id = "map";

  for(unsigned int i = 0; i < ids.size(); i++)
  {
    const uint16_t& current_nid = ids.at(i);
    const uint8_t& current_rssi = rssi_values.at(i);

    ROS_INFO("Got rssi data from node %d with rssi value %d", current_nid, current_rssi);

    if(current_rssi > RSSI_THRESH)
    {
      for(unsigned int i = 0; i < g_heart_beat_list.heartBeatList.size(); i++)
      {
        const kobuki_fleet_msgs::HeartBeat& current_heartbeat = g_heart_beat_list.heartBeatList.at(i);

        if(current_heartbeat.nid == current_nid) // search for matching entry
        {
          if(current_heartbeat.rob_status == kobuki_fleet_msgs::HeartBeat::OK)
          {
            geometry_msgs::Pose pose;

            tf::quaternionTFToMsg(tf::createQuaternionFromYaw(current_heartbeat.yaw), pose.orientation);
            pose.position.x = current_heartbeat.x;
            pose.position.y = current_heartbeat.y;
            pose.position.z = current_rssi;     // use z value for transferring rssi value

            poses.poses.push_back(pose);
          }

          break; // each id is unique
        }
      }
    }
  }

  if(poses.poses.size() > 0)
  {
    g_pub_poses.publish(poses);
  }
}

void calHeartbeatList(const kobuki_fleet_msgs::HeartBeatListConstPtr& msg)
{
  g_heart_beat_list = *msg;
}




