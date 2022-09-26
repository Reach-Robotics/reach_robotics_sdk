#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "packetID.h"
// "Please install the Reach Robotics SDK Critical Components. See https://reach-robotics.github.io/reach_robotics_sdk/bplprotocol/sdk_install.html"

#ifndef BPL_PROTOCOL_H
#define BPL_PROTOCOL_H

#define MAX_PACKET_LENGTH 64
#define HEADER_SIZE 4


/**
 * @brief BPL Packet Structure 
 * 
 */
struct Packet {
    uint8_t packetID;
    uint8_t deviceID;
    uint8_t data[MAX_PACKET_LENGTH];
    uint8_t dataLength;
};

uint8_t cobs_encode( uint8_t* input, uint8_t length, uint8_t * output)
{
	uint8_t read_index = 0;
	uint8_t write_index = 1;
	uint8_t code_index = 0;
    uint8_t code = 1;

    while(read_index < length) {
        if(input[read_index] == 0) {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            output[write_index++] = input[read_index++];
            code++;
            if(code == 0xFF) {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    output[code_index] = code;
    return write_index;
}

uint8_t cobs_decode(const uint8_t* input, uint8_t length, uint8_t* output) {
	uint8_t read_index = 0;
	uint8_t write_index = 0;
    uint8_t code;
    uint8_t i;

    while(read_index < length) {
        code = input[read_index];
        if(read_index + code > length && code != 1) return 0;
        read_index++;
        for(i = 1; i < code; i++) output[write_index++] = input[read_index++];
        if(code != 0xFF && read_index != length)
        	output[write_index++] = '\0';
    }
    return write_index;
}

static unsigned char const crc8_table[] = {
    0xea, 0xd4, 0x96, 0xa8, 0x12, 0x2c, 0x6e, 0x50, 0x7f, 0x41, 0x03, 0x3d,
    0x87, 0xb9, 0xfb, 0xc5, 0xa5, 0x9b, 0xd9, 0xe7, 0x5d, 0x63, 0x21, 0x1f,
    0x30, 0x0e, 0x4c, 0x72, 0xc8, 0xf6, 0xb4, 0x8a, 0x74, 0x4a, 0x08, 0x36,
    0x8c, 0xb2, 0xf0, 0xce, 0xe1, 0xdf, 0x9d, 0xa3, 0x19, 0x27, 0x65, 0x5b,
    0x3b, 0x05, 0x47, 0x79, 0xc3, 0xfd, 0xbf, 0x81, 0xae, 0x90, 0xd2, 0xec,
    0x56, 0x68, 0x2a, 0x14, 0xb3, 0x8d, 0xcf, 0xf1, 0x4b, 0x75, 0x37, 0x09,
    0x26, 0x18, 0x5a, 0x64, 0xde, 0xe0, 0xa2, 0x9c, 0xfc, 0xc2, 0x80, 0xbe,
    0x04, 0x3a, 0x78, 0x46, 0x69, 0x57, 0x15, 0x2b, 0x91, 0xaf, 0xed, 0xd3,
    0x2d, 0x13, 0x51, 0x6f, 0xd5, 0xeb, 0xa9, 0x97, 0xb8, 0x86, 0xc4, 0xfa,
    0x40, 0x7e, 0x3c, 0x02, 0x62, 0x5c, 0x1e, 0x20, 0x9a, 0xa4, 0xe6, 0xd8,
    0xf7, 0xc9, 0x8b, 0xb5, 0x0f, 0x31, 0x73, 0x4d, 0x58, 0x66, 0x24, 0x1a,
    0xa0, 0x9e, 0xdc, 0xe2, 0xcd, 0xf3, 0xb1, 0x8f, 0x35, 0x0b, 0x49, 0x77,
    0x17, 0x29, 0x6b, 0x55, 0xef, 0xd1, 0x93, 0xad, 0x82, 0xbc, 0xfe, 0xc0,
    0x7a, 0x44, 0x06, 0x38, 0xc6, 0xf8, 0xba, 0x84, 0x3e, 0x00, 0x42, 0x7c,
    0x53, 0x6d, 0x2f, 0x11, 0xab, 0x95, 0xd7, 0xe9, 0x89, 0xb7, 0xf5, 0xcb,
    0x71, 0x4f, 0x0d, 0x33, 0x1c, 0x22, 0x60, 0x5e, 0xe4, 0xda, 0x98, 0xa6,
    0x01, 0x3f, 0x7d, 0x43, 0xf9, 0xc7, 0x85, 0xbb, 0x94, 0xaa, 0xe8, 0xd6,
    0x6c, 0x52, 0x10, 0x2e, 0x4e, 0x70, 0x32, 0x0c, 0xb6, 0x88, 0xca, 0xf4,
    0xdb, 0xe5, 0xa7, 0x99, 0x23, 0x1d, 0x5f, 0x61, 0x9f, 0xa1, 0xe3, 0xdd,
    0x67, 0x59, 0x1b, 0x25, 0x0a, 0x34, 0x76, 0x48, 0xf2, 0xcc, 0x8e, 0xb0,
    0xd0, 0xee, 0xac, 0x92, 0x28, 0x16, 0x54, 0x6a, 0x45, 0x7b, 0x39, 0x07,
    0xbd, 0x83, 0xc1, 0xff};

// Return the CRC-8 of data[0..len-1] applied to the seed crc. This permits the
// calculation of a CRC a chunk at a time, using the previously returned value
// for the next seed. If data is NULL, then return the initial seed. See the
// test code for an example of the proper usage.
unsigned crc8(unsigned crc, unsigned char const *data, size_t len)
{
    if (data == NULL)
        return 0;
    crc &= 0xff;
    unsigned char const *end = data + len;
    while (data < end)
        crc = crc8_table[crc ^ *data++];
    return crc;
}

/**
 * @brief Encode a packet
 * 
 * @param packet_buffer Buffer for packet to be filled into.
 * @param deviceID Device ID to Encode
 * @param packetID Packet ID to Encode
 * @param data data buffer to Encode
 * @param length length of data buffer
 * @return * size_t The size of the packet
 */
size_t encodePacketBare(uint8_t* packet_buffer, uint8_t deviceID, uint8_t packetID, uint8_t* data, size_t length)
{
    uint8_t tempBuffer[MAX_PACKET_LENGTH];
    memset(tempBuffer, 0, MAX_PACKET_LENGTH);
    size_t totalLength = length + HEADER_SIZE;
    memcpy(tempBuffer, data, length);
    tempBuffer[totalLength-4] = packetID;
    tempBuffer[totalLength-3] = deviceID;
    tempBuffer[totalLength-2] = totalLength;
    tempBuffer[totalLength-1] = crc8(0xff, tempBuffer, totalLength-1);
    size_t packetLength = cobs_encode(tempBuffer, totalLength, packet_buffer);

    return (size_t) packetLength;

}

/**
 * @brief Encode a Packet using a packet struct.
 * 
 * @param packet_buffer Buffer for the packet to be filled into.
 * @param packet Packet Structure to Encode.
 * @return size_t Length of the packet.
 */
size_t encodePacket(uint8_t* packet_buffer, struct Packet* packet)
{
    uint8_t tempBuffer[MAX_PACKET_LENGTH];
    memset(tempBuffer, 0, MAX_PACKET_LENGTH);
    size_t length = packet->dataLength;

    size_t totalLength = length + HEADER_SIZE;
    memcpy(tempBuffer, packet->data, length);
    tempBuffer[totalLength-4] = packet->packetID;
    tempBuffer[totalLength-3] = packet->deviceID;
    tempBuffer[totalLength-2] = totalLength;
    tempBuffer[totalLength-1] = crc8(0xff, tempBuffer, totalLength-1);
    size_t packetLength = cobs_encode(tempBuffer, totalLength, packet_buffer);
    return packetLength;
}

/**
 * @brief Encodes a single float into 4 Bytes.
 * 
 * @param outputBuffer Output buffer to encoded bytes to fill.
 * @param f Float to encode.
 * @return size_t Length of encoded floats (4).
 */
size_t encodeFloat(uint8_t* outputBuffer, float f)
{
    uint32_t asInt = *((uint32_t*)&f);
    int i;
	for (i = 0; i < 4; i++) {
		outputBuffer[i] = (asInt >> 8 * i) & 0xFF;
	}
	return 4;
}

/**
 * @brief Encodes a list of floats.
 * 
 * @param outputBuffer Output buffer to be filled.
 * @param floatList Pointer to a list of floats.
 * @param length Length of the list of floats.
 * @return size_t Length of the encoded bytes. (4 * length).
 */
size_t encodeFloats(uint8_t* outputBuffer, float* floatList, size_t length)
{

    for(int i = 0; i < length; i++)
    {
        float f = floatList[i];
        uint32_t asInt = *((uint32_t*)&f);
        for (int j = 0; j < 4; j++) 
        {
		    outputBuffer[i*4 + j] = (asInt >> 8 * j) & 0xFF;
	    }
    }
    return length*sizeof(uint8_t) * 4;
}

/**
 * @brief Decode Bytes into floats.
 * 
 * @param outputFloatBuffer Pointer to where to decode the floats into.
 * @param inputBuffer Pointer to the list of bytes of the floats.
 * @param inputBufferLength Length of the bytes buffer (should be divisable by 4)
 * @return size_t Amount of floats decoded. (inputBufferLength/4)
 */
size_t decodeFloats(float* outputFloatBuffer, uint8_t* inputBuffer, size_t inputBufferLength)
{
    size_t floatLength = inputBufferLength/4;
    uint8_t* inputBufferPointer;
    uint32_t asInt;
    for(int i = 0; i < floatLength; i++)
    {   
        asInt = 0;
        inputBufferPointer = &inputBuffer[i*4];

        for(int j = 0; j<4; j++)
        {   
            asInt = asInt | (inputBufferPointer[j] << 8 * j);
        }

        outputFloatBuffer[i] = *((float*)&asInt);
    }

    return floatLength;
}

/**
 * @brief Decode a single float
 * 
 * @param inputBuffer Pointer to the bytes list (4 bytes)
 * @return float Single decoded float.
 */
float decodeFloat(uint8_t* inputBuffer)
{
    uint32_t asInt = 0;
    for(int i = 0; i<4; i++)
    {   
        asInt = asInt | (inputBuffer[i] << 8 * i);
    }
    float f = *((float*)&asInt);
    return f;
}

/**
 * @brief Devcodes bytes into a packet.
 * 
 * @param packet Packet to fill data into.
 * @param inputData Pointer to the list of bytes.
 * @param inputDatalength Length of the bytes.
 * @return int Status of the Decoding. Less than zero is a failure to Decode.
 */
int decodePacket(struct Packet* packet, uint8_t* inputData, size_t inputDatalength)
{
    if (inputDatalength > MAX_PACKET_LENGTH){
        // printf("LENGTH: %d is longer than Maximum Packet Length %d\n", inputDatalength, MAX_PACKET_LENGTH);
        return -1;
    }
    
    // uint8_t inputDataCopy[MAX_PACKET_LENGTH];    

    // memcpy(inputDataCopy, inputData, inputDatalength);

    size_t packetEndIndex = 0;
    while(inputData[packetEndIndex] != 0 && packetEndIndex<inputDatalength)
    {
        packetEndIndex++;
    }

    // inputDataCopy[packetEndIndex] = 0xFF;

    uint8_t packetLength = inputData[packetEndIndex-2];

    // Check if length is valid

    if(packetLength < 4 || packetLength >= inputDatalength){
        // Length is incoreect
        packet->packetID = 0;
        packet->deviceID = 0;
        return -2;
    }

    uint8_t unwrapedBuffer[MAX_PACKET_LENGTH];
    memset(unwrapedBuffer, 0, MAX_PACKET_LENGTH);
    memcpy(unwrapedBuffer, inputData, packetEndIndex);

    unwrapedBuffer[packetEndIndex] = 0xFF;

    uint8_t decodedData[MAX_PACKET_LENGTH];
    memset(decodedData, 0, MAX_PACKET_LENGTH);

    cobs_decode(unwrapedBuffer, packetLength+1, decodedData);    

    uint8_t crc = decodedData[packetLength-1];
    uint8_t deviceID = decodedData[packetLength-3];
    uint8_t packetID = decodedData[packetLength-4];

    uint8_t crcCheck=crc8(0xff, decodedData, packetLength-1);

    if (crc != crcCheck){
        // CRC is not correct
        return -3;
    }

    packet->dataLength = packetLength-4;
    packet->deviceID = deviceID;
    packet->packetID = packetID;
    memset(packet->data, 0 ,MAX_PACKET_LENGTH);
    memcpy(packet->data, decodedData, packetLength-4);

    return 1; 
}

#endif  /* BPL_PROTOCOL_H */