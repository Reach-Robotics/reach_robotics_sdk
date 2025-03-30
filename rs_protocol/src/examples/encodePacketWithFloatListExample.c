/*
 * @file encodePacketWithFloatListExample.c
 * @author Reach Robotics PTY LTD
 * 
 * @brief This example demonstrates how to encode a packet containing a list of floating-point numbers
 * using the rs-protocol.
 */
#include <stdio.h>
#include "../rs_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void main(void)
{

	uint8_t encodedPacket[MAX_PACKET_LENGTH];
	memset(encodedPacket, 0, MAX_PACKET_LENGTH);

	uint8_t deviceID = 0x01;
	uint8_t packetID = POSITION;
	float floatList[5] = {1.2, 2.3, 3.4, 4.5, 5.6};

	uint8_t encodedFloatData[4 * 5];
	size_t dataLength = encodeFloats(encodedFloatData, floatList, 5);

	struct Packet packet;
	packet.deviceID = deviceID;
	packet.packetID = packetID;
    packet.dataLength = dataLength;
	memcpy(packet.data, encodedFloatData, dataLength);

	size_t packetLength = encodePacket(encodedPacket, &packet);

	printf("Encoded Packet: ");
	for (int i = 0; i <= packetLength; i++) {
        printf(" %d", encodedPacket[i]);
    }
    printf("\n")
}

#ifdef __cplusplus
}
#endif