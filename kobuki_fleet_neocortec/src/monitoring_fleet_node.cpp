/*
 * monitoring_fleet_node.cpp
 *
 *  Created on: Aug 5, 2016
 *      Author: amndan
 */

#include <ros/ros.h>
#include <kobuki_fleet_msgs/HeartBeat.h>
#include <kobuki_fleet_msgs/HeartBeatList.h>
#include <kobuki_fleet_msgs/ConnectionState.h>
#include <neocortec/neocortecData.h>
#include <neocortec/neocortecSendData.h>
//#include "roboter_states.h"
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

enum roboterStatus {
ROBOTER_STATUS_OK = 0,
ROBOTER_STATUS_NO_CONNECTION};


/**
 * @brief calback function for heartbeat message from all robots
 * @param message container
 */
void calHeartbeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg);

/**
 * @brief Timer for processing and publishing heartbeats
 * @param event ros timer event
 */
void calMaintainHBListTimer(const ros::TimerEvent& event);

/**
 * @brief Timer for detecting network problems and publishing network state
 * @param event ros timer event
 */
void calCheckNetworkStateTimer(const ros::TimerEvent& event);

/**
 * @brief Timer for sending global heartbeat message
 * @param event ros timer event
 */
void calSendHeartbeatTimer(const ros::TimerEvent& event);

/**
 * Variables
 */
std::string g_top_heartbeat;
std::string g_top_heartbeat_list;
std::string g_top_connection_state;
std::string g_top_send_nc_data;
std::string g_map_frame;
std::string g_footprint_frame;
bool g_send_heartbeat;

ros::Subscriber g_sub_heartbeat;

ros::Publisher g_pub_heartbeat;
ros::Publisher g_pub_heartbeat_list;
ros::Publisher g_pub_connection_status;

ros::ServiceClient g_cli_send_nc_data;

ros::Timer g_tim_maintain_hb_list;
ros::Timer g_tim_check_network_state;
ros::Timer g_tim_send_heartbeat;

kobuki_fleet_msgs::HeartBeatList g_heartbeat_list;
ros::Time g_last_received_hb_stamp(0);
ros::Time g_last_sent_hb_stamp(0);

kobuki_fleet_msgs::ConnectionState g_connection_state;

uint16_t g_my_roboter_id;
uint16_t g_my_neocortec_id;
uint8_t g_neocortec_rssi_port;

/**
 * @brief fleet monitoring node processes heartbeats, monitors alive robots in a local heartbeatlist and monitors network status
 * @param argc
 * @param argv
 * @return main returnvalue
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "monitoring_fleet_node");

  ros::NodeHandle nh = ros::NodeHandle();
  ros::NodeHandle prv_nh = ros::NodeHandle("~");

  ROS_INFO("Started monitoring fleet node. Set connection state to WiFi for startup.");
  g_connection_state.state = g_connection_state.WIFI; //assume wifi for correct initialization

  int tmp_int = 0;
  prv_nh.param<std::string>("topic_heartbeat_global", g_top_heartbeat, "/HB");
  prv_nh.param<std::string>("topic_heartbeat_list", g_top_heartbeat_list, "heart_beat_list");
  prv_nh.param<std::string>("topic_connection_state", g_top_connection_state, "connection_state");
  prv_nh.param<std::string>("topic_neocortec_send_data", g_top_send_nc_data, "neocortec_00_send_data");
  prv_nh.param<std::string>("frame_map", g_map_frame, "map");
  prv_nh.param<std::string>("frame_base_footprint", g_footprint_frame, "base_footprint");
  prv_nh.param<bool>("send_heartbeat", g_send_heartbeat, true);
  prv_nh.param<int>("roboter_id", tmp_int, 1);
  g_my_roboter_id = tmp_int;
  prv_nh.param<int>("neocortec_id", tmp_int, 1);
  g_my_neocortec_id = tmp_int;
  prv_nh.param<int>("neocortec_port", tmp_int, 1);
  g_neocortec_rssi_port = tmp_int;

  ROS_ASSERT(g_top_heartbeat.at(0) == "/");             // must be global ns
  ROS_ASSERT(g_top_heartbeat_list.at(0) != "/");        // must not be global ns

  // publishers, subscribers, services and timers

  g_sub_heartbeat = nh.subscribe(g_top_heartbeat, 10, calHeartbeat);
  g_pub_heartbeat_list = nh.advertise<kobuki_fleet_msgs::HeartBeatList>(g_top_heartbeat_list, 1);
  g_tim_maintain_hb_list = nh.createTimer(ros::Duration(1.0), calMaintainHBListTimer);

  if(g_send_heartbeat)
  {
    g_pub_connection_status = nh.advertise<kobuki_fleet_msgs::ConnectionState>(g_top_connection_state, 1);
    g_pub_heartbeat = nh.advertise<kobuki_fleet_msgs::HeartBeat>(g_top_heartbeat, 1);
    g_cli_send_nc_data = nh.serviceClient<neocortec::neocortecSendData>(g_top_send_nc_data);
    g_tim_check_network_state = nh.createTimer(ros::Duration(1.0), calCheckNetworkStateTimer);
    g_tim_send_heartbeat = nh.createTimer(ros::Duration(0.5), calSendHeartbeatTimer);
  }

  ros::spin();

  return EXIT_SUCCESS;
}

void calHeartbeat(const kobuki_fleet_msgs::HeartBeatConstPtr& msg)
{
  ros::Time now = ros::Time::now();

  ROS_DEBUG("Received HeardBeat from roboter id %d", msg->rid);

  // search for multible heartbeats and keep just the new one
  for (unsigned int i = 0; i < g_heartbeat_list.heartBeatList.size(); i++)
  {
    if (g_heartbeat_list.heartBeatList.at(i).rid == msg->rid)
    {
      g_heartbeat_list.heartBeatList.erase(g_heartbeat_list.heartBeatList.begin() + i);
    }
  }

  g_heartbeat_list.heartBeatList.push_back(*msg);

  g_heartbeat_list.heartBeatList.back().header.stamp = now; // reset timestamp to avoid timesync problems

  if(g_heartbeat_list.heartBeatList.back().rid != g_my_roboter_id) // if this was another robot
  {
    g_last_received_hb_stamp = now;
  }
}

void calMaintainHBListTimer(const ros::TimerEvent& event)
{
  ros::Time now = ros::Time::now();

  for (unsigned int i = 0; i < g_heartbeat_list.heartBeatList.size(); i++) // all entries
  {
    if (g_heartbeat_list.heartBeatList.at(i).rob_status != ROBOTER_STATUS_NO_CONNECTION)  // just for connected nodes
    {
      ros::Duration time_diff = now - g_heartbeat_list.heartBeatList.at(i).header.stamp;
      if (g_heartbeat_list.heartBeatList.at(i).nchb) // is neocortec
      {
        if (time_diff > ros::Duration(5.0)) // expired?
        {
          ROS_INFO("MFN: Lost connection to roboterID %d. Last connection via neocortec.",
                   g_heartbeat_list.heartBeatList.at(i).rid);
          g_heartbeat_list.heartBeatList.at(i).rob_status = ROBOTER_STATUS_NO_CONNECTION;
        }
      }
      else  // is wifi connection
      {
        if (time_diff > ros::Duration(2.0)) // expired?
        {
          ROS_INFO("MFN: Lost connection to roboterID %d. Last connection via wifi",
                   g_heartbeat_list.heartBeatList.at(i).rid);
          g_heartbeat_list.heartBeatList.at(i).rob_status = ROBOTER_STATUS_NO_CONNECTION;
        }
      }
    }
  }
  g_pub_heartbeat_list.publish(g_heartbeat_list);
}

void calCheckNetworkStateTimer(const ros::TimerEvent& event)
{
  ros::Time now = ros::Time::now();

  if(g_last_received_hb_stamp == ros::Time(0) || g_last_sent_hb_stamp == ros::Time(0))
  {
    ROS_WARN_ONCE("Waiting for receiving and sending at least one heartbeat msg for initialization");
    return;
  }
  else
    ROS_INFO_ONCE("First heartbeat msg heard! initializing...");

  bool send_expired = now - g_last_sent_hb_stamp >= ros::Duration(30.0);
  bool receive_expired = now - g_last_received_hb_stamp >= ros::Duration(5.0);

  if(send_expired)
  {
    if(g_connection_state.state != g_connection_state.DISCONNECTED)
        ROS_INFO("send flag expired -> setting connection state to DISCONNECTED");

    g_connection_state.state = g_connection_state.DISCONNECTED;
    g_pub_connection_status.publish(g_connection_state);
    return;
  }

  if(receive_expired)
  {
    if(g_connection_state.state != g_connection_state.NEOCORTEC)
        ROS_INFO("receive flag expired -> setting connection state to NEOCORTEC");

    g_connection_state.state = g_connection_state.NEOCORTEC;
    g_pub_connection_status.publish(g_connection_state);
    return;
  }
  else
  {
    if(g_connection_state.state != g_connection_state.WIFI)
        ROS_INFO("recover connection -> setting connection state to WIFI");

    g_connection_state.state = g_connection_state.WIFI;
    g_pub_connection_status.publish(g_connection_state);
    return;
  }


}

void calSendHeartbeatTimer(const ros::TimerEvent& event)
{
  ros::Time now = ros::Time::now();

  tf::StampedTransform tf_pose;
  tf::TransformListener tf_listener;

  float x = 0.0;
  float y = 0.0;
  float yaw = 0.0;

  if(tf_listener.waitForTransform(g_map_frame, g_footprint_frame, ros::Time(0), ros::Duration(1.0)))
  {
    tf_listener.lookupTransform(g_map_frame, g_footprint_frame, ros::Time(0), tf_pose);
    x = tf_pose.getOrigin().getX();
    y = tf_pose.getOrigin().getY();
    yaw = tf::getYaw(tf_pose.getRotation());
  }
  else
  {
    ROS_DEBUG("Could not lookup transform for robots pose --> set pose to 0 0 0");
  }

  if(g_connection_state.state == g_connection_state.WIFI)
  {
    ROS_DEBUG("send wifi heartbeat...");
    kobuki_fleet_msgs::HeartBeat hb;
    hb.header.stamp = now;
    hb.header.frame_id = "map";
    hb.nchb = false;
    hb.rid = g_my_roboter_id;
    hb.nid = g_my_neocortec_id;
    hb.rob_status = ROBOTER_STATUS_OK;
    hb.x = x;
    hb.y = y;
    hb.yaw = yaw;
    g_pub_heartbeat.publish(hb);

    g_last_sent_hb_stamp = now;

    return;
  }

  /*Neocortec:
    if(g_connection_state.state == g_connection_state.NEOCORTEC)
  {
    ROS_DEBUG("send neocortec heartbeat...");
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

    neocortec::neocortecData nc_msg;
    uint8_t nc_data[16];

    // serialize heartbeat
    memcpy(nc_data + 0, &g_my_roboter_id, sizeof(g_my_roboter_id));
    *(nc_data + 2) = (uint8_t) ROBOTER_STATUS_OK;
    memcpy(nc_data + 3, &x, sizeof(x));
    memcpy(nc_data + 7, &y, sizeof(y));
    memcpy(nc_data + 11, &yaw, sizeof(yaw));
    *(nc_data + 15) = (uint8_t) true; // nchb is true

    // copy c array to msg's data vector
    for(unsigned int i = 0; i < 16; i++)
    {
      nc_msg.data.push_back(nc_data[i]);
    }

    neocortec::neocortecSendData srv;
    srv.request.data = nc_msg.data; // TODO: use neocortec msg in neocortec srv
    srv.request.port = g_neocortec_rssi_port;

    // try to send nc heartbeat to another node in network
    for(unsigned int i = 0; i < g_heartbeat_list.heartBeatList.size(); i++)
    {
      uint16_t curr_rid = g_heartbeat_list.heartBeatList.at(i).rid;
      uint8_t curr_nid = g_heartbeat_list.heartBeatList.at(i).nid;

      // catch own heartbeat
      if(curr_rid == g_my_roboter_id)
      {
        continue;
      }

      // set id
      srv.request.nodeId = curr_nid;

      if (g_cli_send_nc_data.call(srv))
      {
        ROS_DEBUG("called neocortec send data srv with id %d", curr_rid);

        if(srv.response.returnValue == true)
        {
          ROS_DEBUG("send nc heartbeat successfully to node id %d", curr_rid);
          g_last_sent_hb_stamp = now;
          return;
        }
        else
        {
          ROS_DEBUG("cannot send nc heartbeat to node id %d", curr_rid);
          continue;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service send nc data");
        continue;
      }
    }
    ROS_WARN("cannot send neocortec hb to any node in my heartbeatlist");
    return;
  }*/
}
