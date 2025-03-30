/*
 * @file encodePacketWithFloatListExample.c
 * @author Reach Robotics PTY LTD
 * 
 * @brief This example demonstrates how to encode a packet containing a single float value 
 * using the rs-protocol.
 */
#include <stdio.h>
#include "../rs_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

void main(void) {
    uint8_t encodedPacket[MAX_PACKET_LENGTH];
    memset(encodedPacket, 0, MAX_PACKET_LENGTH);

    uint8_t deviceID = 0x01;
    uint8_t packetID = POSITION;
    float f = 4.123;

    uint8_t encodedFloatData[4];
    size_t dataLength = encodeFloat(encodedFloatData, f);

    struct Packet packet;
    packet.deviceID = deviceID;
    packet.packetID = packetID;
    memcpy(packet.data, encodedFloatData, dataLength);
    packet.dataLength = dataLength;

    size_t packetLength = encodePacket(encodedPacket, &packet);

    printf("Encoded Packet: ");
    for (int i = 0; i <= packetLength; i++) {
        printf(" %d", encodedPacket[i]);
    }
    printf("\n");
}

#ifdef __cplusplus
}
#endif