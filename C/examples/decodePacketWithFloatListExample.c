/* 
C/examples/decodePacketWithFloatListExample.c

Author: John Sumskas
Email: j.sumskas@reachrobotics.com
Date: 21/06/2022

*/
#include <stdio.h>
#include "../bplprotocol.h"

#define PACKET_LENGTH 26

void main(void){

    /* **************************** encodePacket() Example *************************************** */
    // create a buffer of bytes for your packet to be filled in with.
    uint8_t encodedPacket[PACKET_LENGTH] = {13, 154, 153, 153, 63, 51, 51, 19, 64, 154, 153, 89, 64, 1, 11, 144, 64, 51, 51, 179, 64, 3, 1, 24, 145, 0};
    // Set data to zeros
    // memset(encodedPacket, 0, MAX_PACKET_LENGTH);   

    // Encoding data with the following information
    struct Packet packet;

    // devode the packet. 
    int result = decodePacket(&packet, encodedPacket, PACKET_LENGTH);

    if (result == -1)
    {
        printf("decodePacket ERROR: LENGTH: %d is longer than Maximum Packet Length %d\n", PACKET_LENGTH, MAX_PACKET_LENGTH);
        return;
    }
    else if (result == -2)
    {
        printf("decodePacket ERROR: Read Packet Length is invalid.\n");
        return;
    }

    else if (result == -3)
    {
        printf("decodePacket ERROR: CRC Check did not Match.\n");
        return;
    }
    else if (result < 0)
    {
        printf("decodePacket ERROR: Unknown Error.\n");
        return;
    }

    printf("Decoded packet: \n");
    printf("Device ID: %d \n", packet.deviceID);
    printf("Packet ID: %d \n", packet.packetID);
    printf("Packet data: ");
    for (int i = 0; i<packet.dataLength; i++)
    {
        printf(" %d", packet.data[i]);
    }
    printf("\nDecoding data to floats \n");

    float floatList[MAX_PACKET_LENGTH/4];

    size_t floatLength = decodeFloats(floatList, packet.data, packet.dataLength);

    printf("Decoded Floats: ");

    for (int i = 0; i<floatLength; i++)
    {
        printf(" %f", floatList[i]);
    }
    printf("\n");
}