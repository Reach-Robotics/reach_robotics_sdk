import re
import struct
from cobs import cobs
from crcmod import crcmod
import logging

try:
    from .packetID import PacketID
except ModuleNotFoundError:
    raise ImportError("Please install the Reach Robotics SDK Critical Components. See https://reach-robotics.github.io/reach_robotics_sdk/bplprotocol/sdk_install.html")
    
logger = logging.getLogger(__name__)



CRC8_FUNC = crcmod.mkCrcFun(0x14D, initCrc=0xFF, xorOut=0xFF)

class BPLProtocol:
    """Class used to encode and decode BPL packets."""

    @staticmethod
    def packet_splitter(buff):
        """
        Split packets coming in along bpl protocol, Packets are split at b'0x00'.

        :param buff: input buffer of bytes
        :return: List of bytes separated by 0x00, and a remaining bytes of an incomplete packet.
        """
        incomplete_packet = None
        packets = re.split(b'\x00', buff)
        if buff[-1] != b'0x00':
            incomplete_packet = packets.pop()
        return packets, incomplete_packet

    @staticmethod
    def parse_packet(packet_in):
        """
        Parse the packet returning a tuple of [int, int, bytes].
        If unable to parse the packet, then return 0,0,b''.
        :param packet_in: bytes of a full packet
        :return: device_id, packet_id, data in bytes.
        """

        packet_in = bytearray(packet_in)

        if packet_in and len(packet_in) > 3:
            try:
                decoded_packet = bytearray(cobs.decode(bytes(packet_in)))
            except cobs.DecodeError as e:
                print("parse_packet(): Cobs Decoding Error, {}".format(e), )
                return 0, 0, bytearray()

            if decoded_packet[-2] != len(decoded_packet):
                print("parse_packet(): Incorrect length: length is {} in {}".format(len(decoded_packet), [str(x) for x in decoded_packet]))
                return 0, 0, bytearray()
            else:
                if CRC8_FUNC(bytes(decoded_packet[:-1])) == decoded_packet[-1]:
                    rx_data = decoded_packet[:-4]

                    device_id = decoded_packet[-3]
                    packet_id = decoded_packet[-4]
                    rx_data = rx_data
                    return device_id, packet_id, rx_data
                else:
                    print("parse_packet(): CRC error in packet ")
                    return 0, 0, bytearray()
        return 0, 0, bytearray()

    @staticmethod
    def encode_packet(device_id, packet_id, data):
        """
         Encode the packet using the bpl protocol.

        :param device_id: Device ID
        :param packet_id: Packet ID
        :param data: Data in bytes
        :return: bytes of the encoded packet.
        """
        tx_packet = bytearray(data)
        tx_packet += bytearray([packet_id, device_id, len(tx_packet)+4])
        tx_packet += bytearray([CRC8_FUNC(bytes(tx_packet))])
        packet = bytearray(cobs.encode(bytes(tx_packet))) + bytearray([0])
        return packet

    @staticmethod
    def decode_floats(data):
        """
        Decode a received byte list, into a float list as specified by the bpl protocol

        Bytes are decoded into 32 bit floats.

        :param data: bytes, but be divisible by 4.
        :return: decoded list of floats
        """
        list_data = list(struct.unpack(str(int(len(data)/4)) + "f", data))
        return list_data

    @staticmethod
    def encode_floats(float_list):
        """ Encode a list of floats into bytes

        Floats are encoded into 32 bits (4 bytes)

        :param float_list: list of floats
        :return: encoded bytes
        """
        data = bytearray(struct.pack('%sf' % len(float_list), *float_list))
        return data

class PacketReader:
    """
    Packet Reader
    Helper class to read and decode incoming bytes and account for the incomplete packets.

    """
    incomplete_packets = bytearray()

    def receive_bytes(self, data):
        """
        Decodes packets.
        Accounts for reading incomplete bytes.

        :param data: input bytes
        :return: a list of of decoded packets (Device ID, Packet ID, data (in bytes))
        """
        # Receive data, and return a decoded packet
        packet_list = []
        encoded_packets, self.incomplete_packets = BPLProtocol.packet_splitter(self.incomplete_packets + data)
        if encoded_packets:
            for encoded_packet in encoded_packets:
                if not encoded_packet:
                    continue
                decoded_packet = BPLProtocol.parse_packet(encoded_packet)
                packet_list.append(decoded_packet)
        return packet_list
