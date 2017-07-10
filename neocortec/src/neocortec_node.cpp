/*
 * neocortec_node.cpp
 *
 *  Created on: Jul 13, 2016
 *      Author: amndan
 */

#include "NcApi.h"
#include <boost/asio.hpp>
#include "assert.h"
#include <iostream>
#include <unistd.h>
#include "serial/serial.h"
#include "ros/ros.h"
#include "neocortec/neocortecData.h"
#include "neocortec/neocortecSendData.h"

#define DEBUG 1
#define MAX_PAYLOAD 19 // max payload is 19 byte

typedef struct OutdatedResponse
{
  uint16_t nodeId;
  ros::Time expiredStamp;
} outdatedResponse;

// Global ROS variables
ros::Publisher g_pubData;
bool g_isAcknowledged = false;
bool g_isNotAcknowledged = false;
bool g_sendSomething = false;
bool g_txError = false;
ros::Duration g_payloadDataTimeToLife = ros::Duration(30);
ros::Duration g_payloadDataTimeToLifeAdditionalTime = ros::Duration(5); //TODO: clean magic numbers
uint16_t g_nodeIdToAcknowledge;
std::vector<OutdatedResponse> g_outdatedResponses; // vector to grab outdated responses when they arrive
ros::Duration g_acknowledgedTimeout; // TODO: set acknowledged timeout via service?

// Global declarations expected by SerialConnection
serial::Serial g_serialHandle;
serial::Timeout g_to = serial::Timeout::simpleTimeout(1000);

// Global declarations expected by NcApi
tNcApi g_ncApi[1];
uint8_t g_numberOfNcApis = 1;
tNcApiRxHandlers g_ncRxHandlers;

// function prototypes
unsigned int readData();
void SetupNcApi(std::string adapter);
void NcApiHostDataHapaCallback(uint8_t n, tNcApiHostDataHapa * p);
void NcApiHostDataCallback(uint8_t n, tNcApiHostData * p);
void NcApiHostNAckCallback(uint8_t n, tNcApiHostAck * p);
void NcApiHostAckCallback(uint8_t n, tNcApiHostAck * p);
void NcApiReadCallback(uint8_t n, uint8_t * msg, uint8_t msgLength);
void NcApiPassthroughRx(uint8_t byte);
void NcApiPassthroughCts();

// ros callback for service
bool srvSendData(neocortec::neocortecSendDataRequest &req, neocortec::neocortecSendDataResponse &res)
{
  /**********************************
   * check payload
   **********************************/
  unsigned int n = req.data.size();

  if (n > MAX_PAYLOAD)
  {
    ROS_ERROR("Neocortec: Max payload size is %d - requested payload was %d.", MAX_PAYLOAD, n);
    res.returnValue = false;
    return true;
  }
  else if (n == 0)
  {
    ROS_ERROR("Neocortec: Payload of size 0 requested - Min Payload is 1.");
    res.returnValue = false;
    return true;
  }

  for (unsigned int i = 0; i < g_outdatedResponses.size(); i++)
  {
    if (g_outdatedResponses.at(i).nodeId == req.nodeId)
    {
      if (g_outdatedResponses.at(i).expiredStamp < ros::Time::now())
      {
        ROS_ERROR("Neocortec: will remove expired outdatedResponse flag from node id %d. "
                  "Normally this schould never happen, only if neocortec modules skip an ACK/NACK",
                  g_outdatedResponses.at(i).nodeId);
        g_outdatedResponses.erase(g_outdatedResponses.begin() + i); // remove it from vector
      }
      else
      {
        ROS_WARN("Neocortec: Waiting for a ACK/NACK bit from requested node. Cannot send message.");
        res.returnValue = false;
        return true;
      }
    }
  }

  /**********************************
   * copy payload to c array
   **********************************/
  uint8_t payload[n];

  for (unsigned int i = 0; i < n; i++) // copy std::vector to c array
  {
    payload[i] = req.data[i];
  }

  /**********************************
   * send payload
   **********************************/
  tNcApiSendParams args;
  args.destNodeId = req.nodeId;
  args.destPort = req.port;
  args.payload = payload;
  args.payloadLength = n;
  args.callbackToken = &g_ncApi;

  g_txError = false;
  g_sendSomething = false;

  int err = NcApiSendAcknowledged(0, &args);

  if (err != NCAPI_OK)
  {
    ROS_ERROR("Neocortec: Error while sending. NCAPI error code: %d", err);
    res.returnValue = false;
    return true;
  }

  NcApiCallbackCtsActive(0); // set cts active --> cts management takes place in UART interface

  if (g_txError)
  {
    ROS_ERROR("Neocortec: Error while sending. TX error");
    res.returnValue = false;
    return true;
  }

  /**********************************
   * wait for acknowledged
   **********************************/
  ros::Time startTimer = ros::Time::now();
  g_nodeIdToAcknowledge = req.nodeId;
  g_isAcknowledged = false;
  g_isNotAcknowledged = false;

  while (1)
  {
    readData();

    if (g_isAcknowledged)
    {
      res.returnValue = true;
      g_nodeIdToAcknowledge = 0; // TODO: use a not used node id for resetting this value
      return true; // acknowledged --> ok
    }

    if (g_isNotAcknowledged)
    {
      res.returnValue = false;
      g_nodeIdToAcknowledge = 0; // TODO: use a not used node id for resetting this value
      return true; // acknowledged --> ok
    }

    if (ros::Time::now() - startTimer > g_acknowledgedTimeout)
    {
      OutdatedResponse outRes;
      outRes.expiredStamp = startTimer + g_payloadDataTimeToLife + g_payloadDataTimeToLifeAdditionalTime;
      outRes.nodeId = g_nodeIdToAcknowledge;
      g_outdatedResponses.push_back(outRes); // add this id to skip its ACK signal
      ROS_WARN("Neocortec: Ack Timer Elapsed --> add id to outdatedResponses vector. Vector Size = %d",
               (int ) g_outdatedResponses.size());
      g_nodeIdToAcknowledge = 0; // TODO: use a not used node id for resetting this value
      res.returnValue = false;
      return true; // timer elapsed
    }
    usleep(50000);
  }

  ROS_ERROR("Neocortec: Panic - This should never happen.");
}

// Global functions expected by NcApi
void NcApiTxData(uint8_t n, uint8_t * finalMsg, uint8_t finalMsgLength)
{
  if (DEBUG)
  {
    uint16_t i;
    printf("NcApiTxData: Try to send raw UART data: ");
    for (i = 0; i < finalMsgLength; i++)
      printf("%02x ", finalMsg[i]);
    printf("\n");
  }

  uint8_t wrote = -1;
  wrote = g_serialHandle.write(finalMsg, finalMsgLength);
  if (wrote != finalMsgLength)
  {
    ROS_ERROR("Neocortec: Bytes to write: %d - i wrote %d", finalMsgLength, wrote);
    g_txError = true;
  }
  else
  {
    g_txError = false;
  }
}

void NcApiMessageWrittenCallback(uint8_t n, void * callbackToken, uint8_t * finalMsg, uint8_t finalMsgLength)
{
  if (DEBUG)
  {
    uint16_t i;
    printf("NcApiMessageWrittenCallback: sent raw UART data: ");
    for (i = 0; i < finalMsgLength; i++)
      printf("%02x ", finalMsg[i]);
    printf("\n");
  }
}

// Handle CTS and RX interrupts
void NcApiPassthroughCts()
{
  NcApiCallbackCtsActive(0);
}

void NcApiPassthroughRx(uint8_t byte)
{
  NcApiRxDataReceived(0, byte);
}

void NcApiReadCallback(uint8_t n, uint8_t * msg, uint8_t msgLength)
{
  if (DEBUG)
  {
    uint16_t i;
    printf("NcApiReadCallback: Raw UART data received: ");
    for (i = 0; i < msgLength; i++)
      printf("%02x ", msg[i]);
    printf("\n");
  }
}

void NcApiHostAckCallback(uint8_t n, tNcApiHostAck * p)
{
  if (DEBUG)
  {
    printf("Received ACK from node %04x \r\n", p->originId);
  }

  for (unsigned int i = 0; i < g_outdatedResponses.size(); i++)
  {
    if (g_outdatedResponses.at(i).nodeId == p->originId)
    {
      g_outdatedResponses.erase(g_outdatedResponses.begin() + i); // remove it from vector
      ROS_WARN("Neocortec: Received an outdated ACK from node id %d - clearing it from vector. Vector Size = %d",
               p->originId, (int ) g_outdatedResponses.size());
      return; // jump out because this was an outdated ACK - not the one we are waiting for
    }
  }

  if (p->originId == g_nodeIdToAcknowledge)
  {
    g_isAcknowledged = true;
    ROS_WARN("Neocortec: My Package to node id %d has been successfully sent.", p->originId);
    g_nodeIdToAcknowledge = 0; // TODO: use a not used node id for resetting this value
  }
  else
  {
    ROS_ERROR("Neocortec: Received ACK package from node id %d - awaited node id %d", p->originId,
              g_nodeIdToAcknowledge);
  }
}

void NcApiHostNAckCallback(uint8_t n, tNcApiHostAck * p)
{
  for (unsigned int i = 0; i < g_outdatedResponses.size(); i++)
  {
    if (g_outdatedResponses.at(i).nodeId == p->originId)
    {
      g_outdatedResponses.erase(g_outdatedResponses.begin() + i); // remove it from vector
      ROS_WARN("Neocortec: Received an NACK from node id %d - clearing it from vector. Vector Size = %d", p->originId,
               (int ) g_outdatedResponses.size());
      return; // jump out because this was an outdated NACK - not the one we are waiting for
    }
  }

  if (p->originId == g_nodeIdToAcknowledge)
  {
    g_isNotAcknowledged = true;
    ROS_WARN("Neocortec: My Package to node id %d has NOT been sent.", p->originId);
    g_nodeIdToAcknowledge = 0; // TODO: use a not used node id for resetting this value
  }
  else
  {
    ROS_ERROR("Neocortec: Received NACK package from node id %d - awaited node id %d", p->originId,
              g_nodeIdToAcknowledge);
  }
}

void NcApiHostDataCallback(uint8_t n, tNcApiHostData * p)
{
  if (DEBUG)
  {
    printf("I have received %d bytes from node %04x. They are %dms old\r\n", p->payloadLength, p->originId,
           p->packageAge * 125);
  }

  double ageInSec = (double)p->packageAge * 125.0 / 1000.0;
  ros::Time stamp = ros::Time::now() - ros::Duration(ageInSec); // generate stamp here anyway --> timing

  if (ageInSec > g_acknowledgedTimeout.toSec())
  {
    // in this case the sender should have jumped this package and marked its ACK as outdated
    // threfore we will throw it even though we received it
    ROS_WARN("Neocortec: Received an outdated package (%dms) from node id %d --> ignore it.", p->packageAge * 125,
             p->originId);
    return;
  }
  else
  {
    // timestamp ok --> continue
    neocortec::neocortecData msg;
    std::vector<uint8_t> dataVec;

    for (unsigned int i = 0; i < p->payloadLength; i++)
    {
      dataVec.push_back(p->payload[i]);
    }

    msg.header.stamp = stamp;
    msg.nodeId = p->originId;
    msg.port = p->port;
    msg.data = dataVec;

    g_pubData.publish(msg);
  }
}

void NcApiHostDataHapaCallback(uint8_t n, tNcApiHostDataHapa * p)
{
  ROS_ERROR("Neocortec: received dataHapaCallback???");
}

// Init function for serial communication and ncAPI
void SetupNcApi(std::string adapter)
{
  g_serialHandle.setBytesize(serial::eightbits);
  g_serialHandle.setParity(serial::parity_none);
  g_serialHandle.setStopbits(serial::stopbits_one);
  g_serialHandle.setFlowcontrol(serial::flowcontrol_hardware);
  g_serialHandle.setBaudrate(115200);
  g_serialHandle.setPort(adapter.c_str());
  g_serialHandle.setTimeout(g_to);
  g_serialHandle.open();

  if (g_serialHandle.isOpen())
  {
    ROS_INFO_STREAM("Connected to serial device.");
  }
  else
  {
    ROS_ERROR_STREAM("Cannot connect to serial device - wrong adapter?");
    exit(EXIT_FAILURE);
  }

  tNcApiRxHandlers * rxHandlers = &g_ncRxHandlers;
  memset(rxHandlers, 0, sizeof(tNcApiRxHandlers));
  rxHandlers->pfnReadCallback = NcApiReadCallback;
  rxHandlers->pfnHostAckCallback = NcApiHostAckCallback;
  rxHandlers->pfnHostNAckCallback = NcApiHostNAckCallback;
  rxHandlers->pfnHostDataCallback = NcApiHostDataCallback;
  rxHandlers->pfnHostDataHapaCallback = NcApiHostDataHapaCallback;

  NcApiInit();
}

unsigned int readData()
{
  uint8_t byte;
  unsigned int i = 0;
  int size;

  while (g_serialHandle.available() > 0)
  {
    size = g_serialHandle.read(&byte, 1);

    if (size == 1)
    {
      NcApiPassthroughRx(byte);
      i++;
    }
    else
    {
      ROS_ERROR("Neocortec: Error at reading data from module!");
    }
  }
  return i;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "neocortecNode");

  ros::NodeHandle prvNh("~");
  ros::NodeHandle nh;

  std::string tmpStr;
  double tmpDbl;

  prvNh.param<std::string>("adapter", tmpStr, "/dev/ttyUSB0");
  SetupNcApi(tmpStr);

  prvNh.param<std::string>("top_srv_send_data", tmpStr, "neocortecSendData");
  ros::ServiceServer srv = nh.advertiseService(tmpStr, srvSendData);

  prvNh.param<std::string>("top_received_data", tmpStr, "neocortecReceivedData");
  g_pubData = nh.advertise<neocortec::neocortecData>(tmpStr, 10);

  prvNh.param<double>("acknowledgedTimeout", tmpDbl, 3);
  g_acknowledgedTimeout = ros::Duration(tmpDbl);

  ros::Rate rate(25);

  while (ros::ok())
  {
    readData();
    //NcApiCallbackCtsActive(0);  // set cts active to send --> cts management takes place in UART interface
    ros::spinOnce();
    rate.sleep();
  }
}

