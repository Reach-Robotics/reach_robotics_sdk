/*
 * @file encodePacketWithFloatListExample.c
 * @author Reach Robotics PTY LTD
 * 
 * @brief This example demonstrates how to decode a packet using the rs-protocol
 * and extract a float value from the decoded data.
 */
#include <stdio.h>
#include "../rs_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_LENGTH 10

void main(void) {
    // create a buffer of bytes for your packet to be filled in with.
    uint8_t encodedPacket[PACKET_LENGTH] = {9, 158, 239, 131, 64, 3, 1, 8, 184, 0};

    struct Packet packet;
    int result = decodePacket(&packet, encodedPacket, PACKET_LENGTH);

    if (result == -1) {
        printf("decodePacket ERROR: LENGTH: %d is longer than Maximum Packet Length %d\n", PACKET_LENGTH, MAX_PACKET_LENGTH);
        return;
    } else if (result == -2) {
        printf("decodePacket ERROR: Read Packet Length is invalid.\n");
        return;
    } else if (result == -3) {
        printf("decodePacket ERROR: CRC Check did not Match.\n");
        return;
    } else if (result < 0) {
        printf("decodePacket ERROR: Unknown Error.\n");
        return;
    }

    printf("Decoded packet: \n");
    printf("Device ID: %d \n", packet.deviceID);
    printf("Packet ID: %d \n", packet.packetID);
    printf("Packet data: ");
    for (int i = 0; i < packet.dataLength; i++) {
        printf(" %d", packet.data[i]);
    }
    printf("\nDecoding data to Float \n");

    float f = decodeFloat(packet.data);

    printf("DecodedFloat: %f", f);
    printf("\n");
}

#ifdef __cplusplus
}
#endif