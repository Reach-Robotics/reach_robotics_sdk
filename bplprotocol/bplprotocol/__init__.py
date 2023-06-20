import re
import struct
from typing import Union, Tuple, List, Optional
from cobs import cobs
from crcmod import crcmod
import logging

try:
    from .packetID import PacketID
except ModuleNotFoundError:
    raise ImportError("Please install the Reach Robotics SDK Critical Components.")



logger = logging.getLogger(__name__)


class BPLProtocol:
    """Class used to encode and decode BPL packets."""
    CRC8_FUNC = crcmod.mkCrcFun(0x14D, initCrc=0xFF, xorOut=0xFF)

    @staticmethod
    def packet_splitter(buff: bytes) -> Tuple[List[bytes], Optional[bytes]]:
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
    def parse_packet(packet_in: Union[bytes, bytearray]) -> Tuple[int, int, bytes]:
        """
        Parse the packet returning a tuple of [int, int, bytes].
        If unable to parse the packet, then return 0,0,b''.
        :param packet_in: bytes of a full packet
        :return: device_id, packet_id, data in bytes.
        """

        packet_in = bytearray(packet_in)

        if packet_in and len(packet_in) > 3:
            try:
                decoded_packet: bytes = cobs.decode(packet_in)
            except cobs.DecodeError as e:
                logger.warning(f"parse_packet(): Cobs Decoding Error, {e}")
                return 0, 0, b''

            if decoded_packet[-2] != len(decoded_packet):
                logger.warning(f"parse_packet(): Incorrect length: length is {len(decoded_packet)} "
                               f"in {[hex(x) for x in list(decoded_packet)]}")
                return 0, 0, b''
            else:
                if BPLProtocol.CRC8_FUNC(decoded_packet[:-1]) == decoded_packet[-1]:
                    rx_data = decoded_packet[:-4]

                    device_id = decoded_packet[-3]
                    packet_id = decoded_packet[-4]
                    rx_data = rx_data
                    return device_id, packet_id, rx_data
                else:
                    logger.warning(f"parse_packet(): CRC error in {[hex(x) for x in list(decoded_packet)]} ")
                    return 0, 0, b''
        return 0, 0, b''

    @staticmethod
    def encode_packet(device_id: int, packet_id: int, data: Union[bytes, bytearray]):
        """
         Encode the packet using the bpl protocol.

        :param device_id: Device ID
        :param packet_id: Packet ID
        :param data: Data in bytes
        :return: bytes of the encoded packet.
        """
        tx_packet = bytes(data)
        tx_packet += bytes([packet_id, device_id, len(tx_packet)+4])
        tx_packet += bytes([BPLProtocol.CRC8_FUNC(tx_packet)])
        packet: bytes = cobs.encode(tx_packet) + b'\x00'
        return packet

    @staticmethod
    def decode_floats(data: Union[bytes, bytearray]) -> List[float]:
        """
        Decode a received byte list, into a float list as specified by the bpl protocol

        Bytes are decoded into 32 bit floats.

        :param data: bytes, but be divisible by 4.
        :return: decoded list of floats
        """
        list_data = list(struct.unpack(str(int(len(data)/4)) + "f", data))
        return list_data

    @staticmethod
    def encode_floats(float_list: List[float]) -> bytes:
        """ Encode a list of floats into bytes

        Floats are encoded into 32 bits (4 bytes)

        :param float_list: list of floats
        :return: encoded bytes
        """
        data = struct.pack('%sf' % len(float_list), *float_list)
        return data


class PacketReader:
    """
    Packet Reader
    Helper class to read and decode incoming bytes and account for the incomplete packets.



    """
    incomplete_packets = b''

    def receive_bytes(self, data: bytes) -> List[Tuple[int, int, bytes]]:
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
