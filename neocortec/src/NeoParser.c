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

#include "NeoParser.h"

int NcApiIsValidFrameTrace(uint8_t * buffer, uint16_t position, uint16_t * outStartAt, uint16_t * outLength)
{
	uint16_t start;
	uint8_t len;
	for (start=0; start<position; start++)
	{
		if (!(buffer[start] == 0x55
		   && start < (position-1)
		   && buffer[start+1] == 0xaa))
			continue;
		if ((start+2) >= position)
			continue;
		len = buffer[start+2];
		// TODO : add check for length
		if ((start+len+6) > position)
			continue;
		*outStartAt = start;
		*outLength = len + 6;
		return 1;
	}
	return 0;
}

int NcApiIsValidFrameBootloader(uint8_t * buffer, uint16_t position, uint16_t * outStartAt, uint16_t * outLength)
{
	uint16_t start;
	uint8_t len;
	for (start=0; start<position; start++)
	{
		if (buffer[start] != 0x3e)
			continue;
		if ((start+1) >= position)
			continue;
		len = buffer[start+1];
		if (len<3)
			continue;
		if ((start+len+2) > position)
			continue;
		if (buffer[start+len+1] != 0x21)
			continue;
		*outStartAt = start;
		*outLength = len + 2;
		return 1;
	}
	return 0;
}

int NcApiIsValidFrameApi(uint8_t * buffer, uint16_t position, uint16_t * outStartAt, uint16_t * outLength)
{
	uint16_t start;
	uint8_t len;
	for (start=0; start<position; start++)
	{
		uint8_t byte = buffer[start];
		if (byte!=HostAckEnum
		 && byte!=HostNAckEnum
		 && byte!=HostDataEnum
		 && byte!=HostDataHapaEnum
		 && byte!=WesRequestEnum
		 && byte!=WesStatusEnum
		 && byte!=NodeInfoReplyEnum)		// check if Host* or Wes*
			continue;
		if ((start+1) >= position)
			continue;
		len = buffer[start+1];
		if (len==0
			|| (byte==HostAckEnum && len!=NCAPI_HOSTACK_LENGTH)
			|| (byte==HostNAckEnum && len!=NCAPI_HOSTACK_LENGTH)
			|| (byte==HostDataEnum && len<=NCAPI_HOSTDATA_HEADER_SIZE)
			|| (byte==HostDataHapaEnum && len<=NCAPI_HOSTDATAHAPA_HEADER_SIZE)
			|| (byte==WesRequestEnum && len!=NCAPI_WESREQUEST_LENGTH)
			|| (byte==WesStatusEnum && len!=NCAPI_WESSTATUS_LENGTH)
			|| (byte==NodeInfoReplyEnum && len!=NCAPI_NODEINFOREPLY_LENGTH)
			)
			continue;
		if ((start+len+NCAPI_HOST_PREFIX_SIZE) > position)
			continue;
		*outStartAt = start;
		*outLength = len + NCAPI_HOST_PREFIX_SIZE;
		return 1;
	}
	return 0;
}

int NcApiIsMsgNodeIdUid(uint8_t * buffer, uint16_t startAt, uint16_t length)
{
	return (buffer[startAt]==0x58 && buffer[startAt+1]==(length-NCAPI_HOST_PREFIX_SIZE));
}

int NcApiIsValidApiFrame(uint8_t * buffer, uint16_t position, uint16_t * outStartAt, uint16_t * outLength)
{
	return NcApiIsValidFrameApi(buffer, position, outStartAt, outLength);
}

int NcApiIsValidSysFrame(uint8_t * buffer, uint16_t position, uint16_t * outStartAt, uint16_t * outLength)
{
	return NcApiIsValidFrameTrace(buffer, position, outStartAt, outLength)
		|| NcApiIsValidFrameBootloader(buffer, position, outStartAt, outLength);
}

void NcApiGetMsgAsHostAck(uint8_t * buffer, tNcApiHostAck * p)
{
	uint16_t hiId = buffer[2];
	uint16_t loId = buffer[3];
	p->originId = (hiId << 8) | loId;
}

void NcApiGetMsgAsHostData(uint8_t * buffer, tNcApiHostData * p)
{
	uint16_t hiId = buffer[2];
	uint16_t loId = buffer[3];
	uint16_t hiAge = buffer[4];
	uint16_t loAge = buffer[5];
	uint8_t payloadLen = buffer[1] - NCAPI_HOSTDATA_HEADER_SIZE;
	p->originId = (hiId << 8) | loId;
	p->packageAge = (hiAge << 8) | loAge;
	p->port = buffer[6];
	p->payloadLength = payloadLen;
	p->payload = buffer + NCAPI_HOSTDATA_MIN_LENGTH;
}

void NcApiGetMsgAsHostDataHapa(uint8_t * buffer, tNcApiHostDataHapa * p)
{
	uint16_t hiId = buffer[2];
	uint16_t loId = buffer[3];
	uint32_t age1 = buffer[4];
	uint32_t age2 = buffer[5];
	uint32_t age3 = buffer[6];
	uint32_t age4 = buffer[7];
	uint8_t payloadLen = buffer[1] - NCAPI_HOSTDATAHAPA_HEADER_SIZE;
	p->originId = (hiId << 8) | loId;
	p->packageAge = (age1 << 24) | (age2 << 16) | (age3 << 8) | age4;
	p->port = buffer[8];
	p->payloadLength = payloadLen;
	p->payload = buffer + NCAPI_HOSTDATAHAPA_MIN_LENGTH;
}

void NcApiGetMsgAsWesStatus(uint8_t * buffer, tNcApiWesStatus * p)
{
	p->Status = buffer[2];
}

void NcApiGetMsgAsWesRequest(uint8_t * buffer, tNcApiWesRequest * p)
{
	p->uid[0] = buffer[2];
	p->uid[1] = buffer[3];
	p->uid[2] = buffer[4];
	p->uid[3] = buffer[5];
	p->uid[4] = buffer[6];
	p->cfngValApplication = buffer[7];
}
