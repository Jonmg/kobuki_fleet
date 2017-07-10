/*
 Copyright (c) 2015, NeoCortec A/S
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software
 without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NCAPI_H_
#define NCAPI_H_

#include <stdint.h>



#ifdef __cplusplus
extern "C" {
#endif

#ifndef NCAPI_TXBUFFER_SIZE
#define NCAPI_TXBUFFER_SIZE 32 //!< Default TX buffer size. Can be defined by the application
#define NCAPI_MAX_PAYLOAD_LENGTH (NCAPI_TXBUFFER_SIZE-5)
#endif

#ifndef NCAPI_RXBUFFER_SIZE
#define NCAPI_RXBUFFER_SIZE 255 //!< Default RX buffer size. Can be defined by the application
#endif

//! \brief HostAck or HostNAck package received
typedef struct NcApiHostAck {
	uint16_t originId;
} tNcApiHostAck;

//! \brief HostData package received
typedef struct NcApiHostData {
	uint16_t originId;
	uint8_t port;
	uint8_t payloadLength;
	uint16_t packageAge;
	uint8_t * payload;
} tNcApiHostData;

//! \brief HostDataHapa package received
typedef struct NcApiHostDataHapa {
	uint16_t originId;
	uint8_t port;
	uint8_t payloadLength;
	uint32_t packageAge;
	uint8_t * payload;
} tNcApiHostDataHapa;

//! \brief WesRequest package received
typedef struct NcApiWesRequest {
	uint8_t uid[5];
	uint8_t cfngValApplication;
} tNcApiWesRequest;

//! \brief WesResponse package received
typedef struct NcApiWesResponse {
	uint8_t uid[5];
	uint16_t nodeId;
} tNcApiWesResponse;

typedef enum {
	NCAPI_WES_STOP = 0,             //!< Stop
	NCAPI_WES_STARTSERVER = 1,    	//!< Start server
	NCAPI_WES_REQUESTSTATUS = 2,   	//!< Request status
	NCAPI_WES_STARTCLIENT = 3,   	//!< Start client
} NcApiWesCmdValues;

typedef struct NcApiWesCmdMsg
{
	NcApiWesCmdValues cmd;
} tNcApiWesCmd;

typedef enum {
	NCAPI_WES_STOPPED = 0,              //!< Stopped
	NCAPI_WES_SERVERRUNNING = 1,    	//!< Server is running
	NCAPI_WES_CLIENTRUNNING = 2,    	//!< Client is running
} NcApiWesStatusValues;

//! \brief WesStatus package received
typedef struct NcApiWesStatus {
	uint8_t Status; //!< See NcApiWesStatusValues
} tNcApiWesStatus;

/**
 * \brief Callback from the application into NcApi whenever CTS becomes active
 * @param n Index of tNcApi instance that the CTS interrupt relates to
 */
void NcApiCallbackCtsActive(uint8_t n);

/**
 * \brief Application provied function that NcApi calls if there is any pending data to be written to the UART
 * @param n Index of tNcApi instance that the data should be written to, ie. which UART
 * @param finalMsg Pointer to the buffer
 * @param finalMsgLength Number of bytes to be written
 */
void NcApiTxData(uint8_t n, uint8_t * finalMsg, uint8_t finalMsgLength);

/**
 * \brief Application provided function that NcApi calls after it has succesfully written the message
 * @param n Index of tNcApi instance that message was written to
 * @param callbackToken Application provided context / token / tag
 * @param finalMsg Pointer to the message
 * @param finalMsgLength Message length in bytes
 */
void NcApiMessageWrittenCallback(uint8_t n, void * callbackToken, uint8_t * finalMsg, uint8_t finalMsgLength);

/**
 * \brief Callback from the application into NcApi whenever a byte is received on the UART
 * @param n Index of tNcApi instance that the byte relates to
 * @param byte The byte received
 */
void NcApiRxDataReceived(uint8_t n, uint8_t byte);

/**
 * \brief Application provided function that NcApi calls whenever any valid NeocCortec messages has been received
 * @param n Index of tNcApi instance that the message was received from
 * @param msg Pointer to the message
 * @param msgLength Message length in bytes
 */
typedef void (*pfnNcApiReadCallback)(uint8_t n, uint8_t * msg, uint8_t msgLength);

/**
 * \brief Application provided function that NcApi calls when a HostAck or HostNAck message has been received
 * @param n Index of tNcApi instance that the message was received from
 * @param m Strongly typed message
 */
typedef void (*pfnNcApiHostAckCallback)(uint8_t n, tNcApiHostAck * m);

/**
 * \brief Application provided function that NcApi calls when a HostData message has been received
 * @param n Index of tNcApi instance that the message was received from
 * @param m Strongly typed message
 */
typedef void (*pfnNcApiHostDataCallback)(uint8_t n, tNcApiHostData * m);

/**
 * \brief Application provided function that NcApi calls when a HostDataHapa message has been received
 * @param n Index of tNcApi instance that the message was received from
 * @param m Strongly typed message
 */
typedef void (*pfnNcApiHostDataHapaCallback)(uint8_t n, tNcApiHostDataHapa * m);

/**
 * \brief Application provided function that NcApi calls when a WesRequest message has been received
 * @param n Index of tNcApi instance that the message was received from
 * @param m Strongly typed message
 */
typedef void (*pfnNcApiWesRequestCallback)(uint8_t n, tNcApiWesRequest * m);

/**
 * \brief Application provided function that NcApi calls when a WesStatus message has been received
 * @param n Index of tNcApi instance that the message was received from
 * @param m Strongly typed message
 */
typedef void (*pfnNcApiWesStatusCallback)(uint8_t n, tNcApiWesStatus * m);

//! \brief Set of application callbacks to handle any received messages. Each callback is optional allowing the application to register specific callbacks only for the message types of particular interest.
typedef struct NcApiRxHandlers {
	pfnNcApiReadCallback pfnReadCallback;							//!< Optional callback for all received messages as a byte array
	pfnNcApiHostAckCallback pfnHostAckCallback;						//!< Optional callback for all received HostAck messages
	pfnNcApiHostAckCallback pfnHostNAckCallback;					//!< Optional callback for all received HostNAck messages
	pfnNcApiHostDataCallback pfnHostDataCallback;					//!< Optional callback for all received HostData messages
	pfnNcApiHostDataHapaCallback pfnHostDataHapaCallback;			//!< Optional callback for all received HostDataHapa messages
	pfnNcApiWesRequestCallback pfnNcApiWesRequestCallback;			//!< Optional callback for all received WesRequest messages
	pfnNcApiWesStatusCallback pfnNcApiWesStatusCallback;	//!< Optional callback for all received WesStatus messages
} tNcApiRxHandlers;

//! \brief Parameters for the NcApiSendAcknowledged function
typedef struct NcApiSendParams {
	uint16_t destNodeId;		//!< Destination node ID
	uint8_t destPort;			//!< Destination port
	uint8_t * payload;			//!< Pointer to payload, if any
	uint8_t payloadLength;		//!< PayloadLength Length of payload
	void * callbackToken;		//!< Application provided token / context / tag that it wants to called back with. NcApi does not inspect this parameter, it merely passes it along
} tNcApiSendParams;

//! \brief Parameters for the NcApiWesCmd function
typedef struct NcApiWesCmdParams {
	NcApiWesCmdValues cmd;		//!< WES Command
	void * callbackToken;		//!< Application provided token / context / tag that it wants to called back with. NcApi does not inspect this parameter, it merely passes it along
} tNcApiWesCmdParams;

//! \brief Parameters for the NcApiWesCmd function
typedef struct NcApiWesResponseParams {
	uint8_t uid[5];				//!< UID
	uint16_t nodeId;			//!< NodeId
	void * callbackToken;		//!< Application provided token / context / tag that it wants to called back with. NcApi does not inspect this parameter, it merely passes it along
} tNcApiWesResponseParams;

//! \brief RX and TX buffers for a particular UART. Managed by NcApi and as such the fields are considered internal to NcApi
typedef struct NcApi {
	uint8_t rxBuffer[ NCAPI_RXBUFFER_SIZE];
	uint16_t rxPosition;
	volatile uint8_t txMsgLen;
	uint8_t txBuffer[ NCAPI_TXBUFFER_SIZE];
	void * writeCallbackToken;
} tNcApi;

//! \brief Application defined and configured rx-handlers
extern tNcApiRxHandlers g_ncRxHandlers;

//! \brief Application defined array of NcApi instances in use
extern tNcApi g_ncApi[];

//! \brief Application defined number of elements in the g_ncApi array
extern uint8_t g_numberOfNcApis;

typedef enum {
	NCAPI_OK = 0,              	//!< Success
	NCAPI_ERR_NODEID = 1,      	//!< NodeId cannot be 0
	NCAPI_ERR_DESTPORT = 2,    	//!< Port must be [0..3]
	NCAPI_ERR_PAYLOAD = 3,     	//!< No payload supplied
	NCAPI_ERR_ENQUEUED = 4, 	//!< There is already one pending message waiting to be written to the UART
	NCAPI_ERR_NULLPAYLOAD = 5, 	//!< Payload length supplied but no payload
	NCAPI_ERR_NOARGS = 6      	//!< No arguments pointer
} NcApiErrorCodes;

//! \brief Initializes the allocated instances of tNcApi
void NcApiInit();

/**
 * \brief Sends one CommandAckowledged message
 * @param n Index of tNcApi instance that the message should be sent via
 * @param args Pointer to instance of tNcApiSendParams that holds the parameters
 * @return 0 upon success. Anything else is an error
 */
NcApiErrorCodes NcApiSendAcknowledged(uint8_t n, tNcApiSendParams * args);

/**
 * \brief Sends one WesCmd message
 * @param n Index of tNcApi instance that the message should be sent via
 * @param args Pointer to instance of tNcApiWesCmdParams that holds the parameters
 * @return 0 upon success. Anything else is an error
 */
NcApiErrorCodes NcApiSendWesCmd(uint8_t n, tNcApiWesCmdParams * args);

/**
 * \brief Sends one WesResponse message
 * @param n Index of tNcApi instance that the message should be sent via
 * @param args Pointer to instance of tNcApiWesResponse that holds the parameters
 * @return 0 upon success. Anything else is an error
 */
NcApiErrorCodes NcApiSendWesResponse(uint8_t n, tNcApiWesResponseParams * args);

/**
 * \brief Cancels any enqueued message
 * @param n Index of tNcApi instance where the message should be dequeued
 **/
void NcApiCancelEnqueuedMessage(uint8_t n);

#ifdef __cplusplus
}
#endif

#endif /* NCAPI_H_ */
