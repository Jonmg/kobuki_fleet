/*
 //Copyright (c) 2015, NeoCortec A/S
 //All rights reserved.
 //
 //Redistribution and use in source and binary forms, with or without
 //modification, are permitted provided that the following conditions are met:
 //
 //1. Redistributions of source code must retain the above copyright notice,
 //   this list of conditions and the following disclaimer.
 //
 //2. Redistributions in binary form must reproduce the above copyright notice,
 //   this list of conditions and the following disclaimer in the documentation
 //   and/or other materials provided with the distribution.
 //
 //3. Neither the name of the copyright holder nor the names of its contributors
 //   may be used to endorse or promote products derived from this software
 //   without specific prior written permission.
 //
 //THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 //AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 //IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 //DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 //FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 //DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 //SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 //CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 //OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 //OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 // This example configures one instance of NcApi.
 // It shows a bare minimum implementation of what is required in order to receive and send messages.
 // It is recommened to walk through ExampleSetupNcApi() and ExampleSendAcknowledged() defined at the end of this file.
 */

#include "NcApi.h"
#include <boost/asio.hpp>
#include <string>
#include "assert.h"
#include <iostream>
#include <unistd.h>

// Global declarations expected by NcApi
tNcApi g_ncApi[1];
uint8_t g_numberOfNcApis = 1;
tNcApiRxHandlers g_ncRxHandlers;

boost::asio::io_service g_io;
std::string g_adapter = "/dev/ttyUSB0";
boost::asio::serial_port g_serial(g_io, g_adapter);
unsigned int g_baudrate = 115200;
uint8_t g_byte;

void boostTxHandler( const boost::system::error_code& error, std::size_t bytes_transferred);

// Global functions expected by NcApi
void NcApiTxData(uint8_t n, uint8_t * finalMsg, uint8_t finalMsgLength)
{
  uint16_t i;
  printf("Try to send raw UART data: ");
  for(i = 0; i < finalMsgLength; i++)
    printf("%02x ", finalMsg[i]);
  printf("\n");


  uint8_t wrote = -1;

  boost::asio::async_write(g_serial, boost::asio::buffer(finalMsg, finalMsgLength), boostTxHandler);
}

void NcApiMessageWrittenCallback(uint8_t n, void * callbackToken, uint8_t * finalMsg, uint8_t finalMsgLength)
{
  std::cout << "returned from NcApiTxData function --> now boost has to send the frame..." << std::endl;
  g_io.poll();
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

// Sample implementation of generic message received handler merely providing the bytes.
// Always called before any of the strongly typed message handlers are called.
void NcApiReadCallback(uint8_t n, uint8_t * msg, uint8_t msgLength)
{
  uint16_t i;
  printf("Raw UART data received: ");
  for(i = 0; i < msgLength; i++)
    printf("%02x ", msg[i]);
  printf("\n");
}
void NcApiHostAckCallback(uint8_t n, tNcApiHostAck * p)
{
  printf("My previous package to node node %04x was successfully delivered\r\n", p->originId);
}
void NcApiHostNAckCallback(uint8_t n, tNcApiHostAck * p)
{
  printf("My previous package to node node %04x was not delivered\r\n", p->originId);
}
void NcApiHostDataCallback(uint8_t n, tNcApiHostData * p)
{
  printf("I have received %d bytes from node %04x. They are %dms old\r\n", p->payloadLength, p->originId, p->packageAge
      * 125);
}
void NcApiHostDataHapaCallback(uint8_t n, tNcApiHostDataHapa * p)
{
  printf("I have received %d bytes from node %x4. They are %dms old\r\n", p->payloadLength, p->originId, ((double)p->packageAge)
      / 524.288);
}

// Sample implementation of how to send one message
void NcApiSendPayloadAcknowledged(uint16_t destNodeId, uint8_t port, uint8_t * payload, uint8_t payloadLen)
{
  tNcApiSendParams args;
  args.destNodeId = destNodeId;
  args.destPort = port;
  args.payload = payload;
  args.payloadLength = payloadLen;
  args.callbackToken = &g_ncApi;
  NcApiSendAcknowledged(0, &args);
}

// Sample implementation of how to setup global rx-handlers and initialize NcApi.
void SetupNcApi()
{

  assert(g_serial.is_open());
  g_serial.set_option(boost::asio::serial_port_base::baud_rate(g_baudrate));
  g_serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::hardware));
  g_serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  g_serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  g_serial.set_option(boost::asio::serial_port_base::character_size(8));

  tNcApiRxHandlers * rxHandlers = &g_ncRxHandlers;
  memset(rxHandlers, 0, sizeof(tNcApiRxHandlers));
  rxHandlers->pfnReadCallback = NcApiReadCallback;
  rxHandlers->pfnHostAckCallback = NcApiHostAckCallback;
  rxHandlers->pfnHostNAckCallback = NcApiHostNAckCallback;
  rxHandlers->pfnHostDataCallback = NcApiHostDataCallback;
  rxHandlers->pfnHostDataHapaCallback = NcApiHostDataHapaCallback;

  NcApiInit();
}

void boostRxHandler( const boost::system::error_code& error, std::size_t bytes_transferred)
{
  std::cout << "hit boost Rx handle!" << std::endl;
  NcApiPassthroughRx(g_byte);
  g_serial.async_read_some(boost::asio::buffer(&g_byte, 1), boostRxHandler);
}

void boostTxHandler( const boost::system::error_code& error, std::size_t bytes_transferred)
{
  std::cout << "hit boost tx handle! " << bytes_transferred << " transfered bytes." << std::endl;
}

int main(void)
{
  SetupNcApi();

  tNcApiWesCmdParams msgs;
  msgs.callbackToken = NULL;
  msgs.cmd = NCAPI_WES_REQUESTSTATUS;

  while(1)
  {
    boost::asio::async_read(g_serial, boost::asio::buffer(&g_byte, 1), boostRxHandler);

    long unsigned int count = 0;

    while(1)
    {
      NcApiSendWesCmd(0, &msgs);
      NcApiPassthroughCts();

      usleep(1000000);

      g_io.poll();

      count++;
    }
  }
}

