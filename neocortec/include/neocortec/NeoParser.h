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

#ifndef NEOPARSER_H_
#define NEOPARSER_H_

#include <stdint.h>
#include "NcApi.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum NcApiMessageType
{
	CommandAcknowledgedEnum = 0x03,
    WesCmdEnum = 0x10,
    WesRequestEnum = 0x61,
    WesResponseEnum = 0x11,
    WesStatusEnum = 0x60,
	HostAckEnum = 0x50,
	HostNAckEnum = 0x51,
	HostDataEnum = 0x52,
	HostDataHapaEnum = 0x53,
	NodeInfoReplyEnum = 0x58
} NcApiMessageType;

#define NCAPI_HOST_PREFIX_SIZE 2
#define NCAPI_HOSTACK_LENGTH 2
#define NCAPI_HOSTDATA_HEADER_SIZE 5
#define NCAPI_HOSTDATA_MIN_LENGTH (NCAPI_HOSTDATA_HEADER_SIZE+NCAPI_HOST_PREFIX_SIZE)
#define NCAPI_HOSTDATAHAPA_HEADER_SIZE 7
#define NCAPI_HOSTDATAHAPA_MIN_LENGTH (NCAPI_HOSTDATAHAPA_HEADER_SIZE+NCAPI_HOST_PREFIX_SIZE)
#define NCAPI_WESREQUEST_LENGTH 6
#define NCAPI_WESRESPONSE_LENGTH 7
#define NCAPI_WESCMD_LENGTH 1
#define NCAPI_WESSTATUS_LENGTH 1
#define NCAPI_NODEINFOREPLY_LENGTH 8

/**
 * \brief Determines if the content in the buffer is a valid Api-message
 * @param buffer Received RX-data
 * @param position Number of received bytes
 * @param[out] outStartAt If a message was found, index into buffer where the message begins
 * @param[out] outLength If a message was found, the message length
 * @return 1==true 0==false
 */
int NcApiIsValidApiFrame(uint8_t * buffer, uint16_t position, uint16_t * outStartAt, uint16_t * outLength);

/**
 * \brief Determines if the content in the buffer is a valid System-trace-message or a Bootloader-message
 * @param buffer Received RX-data
 * @param position Number of received bytes
 * @param[out] outStartAt If a message was found, index into buffer where the message begins
 * @param[out] outLength If a message was found, the message length
 * @return 1==true 0==false
 */
int NcApiIsValidSysFrame(uint8_t * buffer, uint16_t position, uint16_t * outStartAt, uint16_t * outLength);

/**
 * \brief Determines whether the message in the buffer contains NodeId and Uid
 * @param buffer Buffer containing message
 * @param startAt Index of message start in buffer
 * @param length Message length
 * @return 1==true 0==false
 */
int NcApiIsMsgNodeIdUid(uint8_t * buffer, uint16_t startAt, uint16_t length);

/**
 * \brief Deserializes the content of the bufer into a tNcApiHostAck instance
 * @param buffer Buffer containing message
 * @param p Instance to deserialize into
 */
void NcApiGetMsgAsHostAck(uint8_t * buffer, tNcApiHostAck * p);

/**
 * \brief Deserializes the content of the bufer into a tNcApiHostData instance
 * @param buffer Buffer containing message
 * @param p Instance to deserialize into
 */
void NcApiGetMsgAsHostData(uint8_t * buffer, tNcApiHostData * p);

/**
 * \brief Deserializes the content of the bufer into a tNcApiHostDataHapa instance
 * @param buffer Buffer containing message
 * @param p Instance to deserialize into
 */
void NcApiGetMsgAsHostDataHapa(uint8_t * buffer, tNcApiHostDataHapa * p);

/**
 * \brief Deserializes the content of the bufer into a tNcApiWesStatus instance
 * @param buffer Buffer containing message
 * @param p Instance to deserialize into
 */
void NcApiGetMsgAsWesStatus(uint8_t * buffer, tNcApiWesStatus * p);

/**
 * \brief Deserializes the content of the bufer into a tNcApiWesRequest instance
 * @param buffer Buffer containing message
 * @param p Instance to deserialize into
 */
void NcApiGetMsgAsWesRequest(uint8_t * buffer, tNcApiWesRequest * p);


#ifdef __cplusplus
}
#endif


#endif /* NEOPARSER_H_ */
