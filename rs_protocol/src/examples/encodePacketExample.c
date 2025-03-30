/*
 * @file encodePacketWithFloatListExample.c
 * @author Reach Robotics PTY LTD
 * 
 * @brief This example demonstrates how to encode a packet using the rs-protocol.
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
    uint8_t packetID = MODE;
    uint8_t data[5] = {1, 2, 3, 4, 5};

    struct Packet packet;
    packet.deviceID = deviceID;
    packet.packetID = packetID;
    memcpy(packet.data, data, 5);
    packet.dataLength = 5;

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