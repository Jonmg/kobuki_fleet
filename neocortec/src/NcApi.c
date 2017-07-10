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

#include <string.h>
#include "NcApi.h"
#include "NeoParser.h"

#ifndef ASSERT
#ifdef DEBUG
extern void __error__(char *pcFilename, uint32_t ui32Line);
#define ASSERT(expr) do                                                       \
                     {                                                        \
                         if(!(expr))                                          \
                         {                                                    \
                             __error__(__FILE__, __LINE__);                   \
                         }                                                    \
                     }                                                        \
                     while(0)
#else
#define ASSERT(expr)
#endif//DEBUG
#endif//ASSERT

tNcApi * NcApiGetInstance(uint8_t n)
{
	ASSERT(n<g_numberOfNcApis);
	return &g_ncApi[n];
}

void NcApiCallbackCtsActive(uint8_t n)
{
	tNcApi * api = NcApiGetInstance(n);
	uint8_t len = api->txMsgLen;
	uint8_t * buffer;
	void * cbToken;
	if (len==0)
		return;
	buffer = api->txBuffer;
	NcApiTxData( n, buffer, len );
	cbToken = api->writeCallbackToken;
	api->writeCallbackToken = 0;
	api->txMsgLen = 0;
	NcApiMessageWrittenCallback( n, cbToken, buffer, len );
}

void NcApiShiftBufferDownAndAdjustPosition(tNcApi * api)
{
	uint16_t remaining = api->rxPosition - 1;
	uint16_t iSrc = 1;
	uint16_t i;
	for (i=0; i<remaining; i++)
		api->rxBuffer[i] = api->rxBuffer[ iSrc + i ];
	api->rxPosition = remaining;
}

void NcApiRxDataReceived(uint8_t n, uint8_t byte)
{
	uint16_t msgStartAt, msgLength;
	tNcApiHostAck ack;
	tNcApiHostData data;
	tNcApiHostDataHapa dataHapa;
	tNcApiWesRequest wesRequest;
	tNcApiWesStatus wesStatus;
	tNcApiRxHandlers * handlers = &g_ncRxHandlers;
	tNcApi * api = NcApiGetInstance(n);
	if (handlers==0)
		return;
	if (api->rxPosition >= NCAPI_RXBUFFER_SIZE)
		NcApiShiftBufferDownAndAdjustPosition(api);
	api->rxBuffer[ api->rxPosition ] = byte;
	api->rxPosition++;
	if (!NcApiIsValidApiFrame(api->rxBuffer, api->rxPosition, &msgStartAt, &msgLength))
		return;
	if (handlers->pfnReadCallback!=0)
		handlers->pfnReadCallback(n, api->rxBuffer + msgStartAt, (uint8_t)(msgLength & 0xff));
	switch (api->rxBuffer[msgStartAt])
	{
		case HostAckEnum:
			if (handlers->pfnHostAckCallback!=0)
			{
				NcApiGetMsgAsHostAck(api->rxBuffer+msgStartAt, &ack);
				handlers->pfnHostAckCallback(n, &ack);
			}
			break;
		case HostNAckEnum:
			if (handlers->pfnHostNAckCallback!=0)
			{
				NcApiGetMsgAsHostAck(api->rxBuffer+msgStartAt, &ack);
				handlers->pfnHostNAckCallback(n, &ack);
			}
			break;
		case HostDataEnum:
			if (handlers->pfnHostDataCallback!=0)
			{
				NcApiGetMsgAsHostData(api->rxBuffer+msgStartAt, &data);
				handlers->pfnHostDataCallback(n, &data);
			}
			break;
		case HostDataHapaEnum:
			if (handlers->pfnHostDataHapaCallback!=0)
			{
				NcApiGetMsgAsHostDataHapa(api->rxBuffer+msgStartAt, &dataHapa);
				handlers->pfnHostDataHapaCallback(n, &dataHapa);
			}
			break;
		case WesStatusEnum:
			if (handlers->pfnNcApiWesStatusCallback!=0)
			{
				NcApiGetMsgAsWesStatus(api->rxBuffer+msgStartAt, &wesStatus);
				handlers->pfnNcApiWesStatusCallback(n, &wesStatus);
			}
			break;
		case WesRequestEnum:
			if (handlers->pfnNcApiWesRequestCallback!=0)
			{
				NcApiGetMsgAsWesRequest(api->rxBuffer+msgStartAt, &wesRequest);
				handlers->pfnNcApiWesRequestCallback(n, &wesRequest);
			}
			break;
	}
	api->rxPosition = 0;
}

void NcApiInit()
{
	uint8_t i;
	for (i=0; i<g_numberOfNcApis; i++)
		memset( (void*)&g_ncApi[i], 0, sizeof(tNcApi) );
}

NcApiErrorCodes NcApiSendAcknowledged
(
	uint8_t n,
	tNcApiSendParams * args
)
{
	uint8_t len;
	uint8_t * buf;
	tNcApi * api = NcApiGetInstance(n);
	if (args==0) return NCAPI_ERR_NOARGS;
	if (args->destNodeId==0) return NCAPI_ERR_NODEID;
	if (args->destPort>4) return NCAPI_ERR_DESTPORT;
	if (args->payloadLength>NCAPI_MAX_PAYLOAD_LENGTH) return NCAPI_ERR_PAYLOAD;
	if (args->payloadLength!=0 && args->payload==0) return NCAPI_ERR_NULLPAYLOAD;
	if (api->txMsgLen!=0) return NCAPI_ERR_ENQUEUED;
	len = 3 + args->payloadLength;
	buf = api->txBuffer;
	buf[0] = CommandAcknowledgedEnum;
	buf[1] = len;
	buf[2] = (args->destNodeId >> 8) & 0xff;
	buf[3] = args->destNodeId  & 0xff;
	buf[4] = args->destPort;
	if (args->payloadLength!=0)
		memcpy( buf+5, args->payload, args->payloadLength );
	api->writeCallbackToken = args->callbackToken;
	api->txMsgLen = 5 + args->payloadLength;
	return NCAPI_OK;
}

NcApiErrorCodes NcApiSendWesCmd
(
	uint8_t n,
	tNcApiWesCmdParams * args
)
{
	uint8_t * buf;
	tNcApi * api = NcApiGetInstance(n);
	buf = api->txBuffer;
	buf[0] = WesCmdEnum;
	buf[1] = NCAPI_WESCMD_LENGTH;
	buf[2] = args->cmd;;
	api->writeCallbackToken = args->callbackToken;
	api->txMsgLen = 3;
	return NCAPI_OK;
}

NcApiErrorCodes NcApiSendWesResponse
(
	uint8_t n,
	tNcApiWesResponseParams * args
)
{
	uint8_t * buf;
	tNcApi * api = NcApiGetInstance(n);
	buf = api->txBuffer;
	buf[0] = WesResponseEnum;
	buf[1] = NCAPI_WESRESPONSE_LENGTH;
	buf[2] = args->uid[0];
	buf[3] = args->uid[1];
	buf[4] = args->uid[2];
	buf[5] = args->uid[3];
	buf[6] = args->uid[4];
	buf[7] = (args->nodeId >> 8) & 0xff;
	buf[8] = args->nodeId  & 0xff;
	api->writeCallbackToken = args->callbackToken;
	api->txMsgLen = 9;
	return NCAPI_OK;
}

void NcApiCancelEnqueuedMessage(uint8_t n)
{
	tNcApi * api = NcApiGetInstance(n);
	api->txMsgLen = 0;
	api->writeCallbackToken = NULL;
}
