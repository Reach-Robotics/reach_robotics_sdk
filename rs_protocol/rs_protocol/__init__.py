import os
import re
import struct
import enum
import ctypes as c
import platform
import serial
import time
import logging
import socket

from typing import Union, Tuple, List, Optional

from .packetID import PacketID
from .mode import Mode

DATA_BYTES_PER_PACKET = 64
RESPONSE_TIMEOUT = 0.001
HALF_DUPLEX_TIMEOUT = 0.1

RS_485_NOTE = "Note: for RS-485 (Half-Duplex) connections, data can not be transmitted and recived simultaniously."

logger = logging.getLogger(__name__)
logger.setLevel('WARNING')
drivers = None


class PacketOption(enum.IntFlag):
    FWD_ALLOW = 0b00000001
    IS_DEMAND = 0b00000010
    PROTOCOL_V2 = 0b00000100


class Packet_t(c.Structure):
    _fields_ = [
        ("length",          c.c_uint8),
        ("address",         c.c_uint8),
        ("code",            c.c_uint16),
        ("crc",             c.c_uint16),
        ("data",            c.c_uint8 * DATA_BYTES_PER_PACKET),
        ("transmitData",    c.c_uint8 * DATA_BYTES_PER_PACKET),
        ("protocol",        c.c_uint8),
        ("option",          c.c_uint8),
        ("useOption",       c.c_uint8),
        ("receiveRegister", c.c_uint16),
        ("totalFrames",     c.c_uint8),
    ]


driver_name = "librs_protocol"
install_path = f"{os.path.dirname(__file__)}/.."
if platform.system() == "Windows" and (platform.processor() == "x86_64" or platform.machine() == 'AMD64'):
    lib_path = f"{install_path}/lib/{driver_name}_windows_x86_64.dll"
elif platform.system() == "Linux" and platform.processor() == "x86_64":
    lib_path = f"{install_path}/lib/{driver_name}_linux_x86_64.so"
elif platform.system() == "Linux" and platform.processor() == "aarch64":
    lib_path = f"{install_path}/lib/{driver_name}_linux_aarch64.so"
elif platform.system() == "Linux":
    lib_path = f"{install_path}/lib/{driver_name}_linux_x86_64.so"


def import_drivers():
    logger.debug(f"Importing rs_protocol driver from: {lib_path}")
    global drivers
    drivers = c.cdll.LoadLibrary(lib_path)


def encode_packet(device_id: int, packet_id: int, data: bytes, option: Optional[int] = None) -> bytes:
    global drivers
    if drivers is None:
        import_drivers()
    p = Packet_t()
    p_pointer = c.byref(p)
    address = c.c_uint8(device_id)
    code = c.c_uint16(packet_id)
    length = c.c_uint8(len(data)+4)

    # Unpack data into buffer
    buffer = (c.c_uint8 * DATA_BYTES_PER_PACKET)(*data)

    # Parse options if required
    p.useOption, c_option = (c.c_uint8(1), c.c_uint8(int(option))) if option is not None else (c.c_uint8(0), c.c_uint8(0))

    # Encode packet
    ret = drivers.coms_encodePacket(p_pointer, address, code, length, c.byref(buffer), c_option)  # type: ignore defined in the driver lib

    if ret == 1:
        return bytes(p.transmitData[0:p.length])
    elif ret == -1:
        logger.warning(f"encode_packet(): length ({length}) is less than PKT_HEADER_LEN ({4})")
        return b''
    elif ret == -2:
        logger.warning(f"encode_packet(): length ({length}) is greater than MAX_PACKET_DATA_SIZE ({DATA_BYTES_PER_PACKET})")
        return b''
    elif ret == -3:
        logger.warning(f"encode_packet(): unsupported packet ID")
        return b''
    else:
        logger.warning(f"encode_packet(): unknown error code")
        return b''


def parse_packet(packet_in: bytes) -> Tuple[int, int, bytes, Optional[int]]:
    global drivers
    if drivers is None:
        import_drivers()
    if len(packet_in) < 4:
        return 0, 0, b'', None

    # Add terminating byte for driver compatibility
    if packet_in[-1] != 0:
        packet_in += b'\x00'

    # HACK: Extract length from end of packet and add CRC and termination to index and drop excess data
    # Shouldn't need to do this, however, have seen some stray bytes in the serial data. 
    buffLength = (packet_in[-3] & 0x7F) + 2
    packetStart = len(packet_in)-buffLength
    if (packetStart < 0):
        logger.warning(f" Missing data in packet. {RS_485_NOTE}")
        return 0, 0, b'', None
    packet_in = packet_in[len(packet_in)-buffLength:]

    # Check if packets in are in a valid range
    if len(packet_in) > DATA_BYTES_PER_PACKET:
        logger.warning(f" packet_in is too large to parse. packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None

    # Generate packet and unpack data into buffer
    p = Packet_t()
    buffer = (c.c_uint8 * DATA_BYTES_PER_PACKET)(*packet_in)

    ret = drivers.coms_decodePacket(c.byref(p), c.byref(buffer), buffLength)  # type: ignore defined in the driver lib

    if ret == 1:
        out_data = bytes([p.data[i] for i in range(p.length - 4)])
        options = p.option if p.useOption else None
        return p.address, p.code, out_data, options
    elif ret == -1:
        logger.warning(f" length ({p.length}) is less than PKT_HEADER_LEN ({4}). packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None
    elif ret == -2:
        logger.warning(f" length ({p.length}) is greater than MAX_PACKET_DATA_SIZE ({DATA_BYTES_PER_PACKET}). packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None
    elif ret == -3:
        logger.warning(f" COBS decoding error. packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None
    elif ret == -4:
        logger.warning(f" CRC error. packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None
    elif ret == -6:
        logger.warning(f" packet does not end with PKT_TERMINATING_BYTE. packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None
    elif ret == -7:
        zeroIndex = len(packet_in) - 1
        length = buffer[zeroIndex-2] & 0x7F    
        logger.warning(f" length ({len(packet_in)}) does not match buff length ({length}). packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None
    else:
        logger.warning(f" unknown error code {ret}. packet_in: {packet_in}. {RS_485_NOTE}")
        return 0, 0, b'', None


def packet_splitter(buff: bytes) -> Tuple[List[bytes], bytes]:
    """ Split packets are delimited by 0x00. """
    incomplete_packet = b''
    packets = re.split(b'\x00', buff)
    if buff[-1] != b'0x00':
        incomplete_packet = packets.pop()
    return packets, incomplete_packet


def decode_floats(data: Union[bytes, bytearray]) -> List[float]:
    """ Decode a received byte list, into a float list as specified by the rr protocol"""
    list_data = list(struct.unpack(str(int(len(data)/4)) + "f", data))
    return list_data


def decode_ints(data: Union[bytes, bytearray]) -> List[int]:
    """ Decode a received byte list, into a float list as specified by the rr protocol"""
    list_data = list(struct.unpack(str(int(len(data))) + "B", data))
    return list_data


def encode_floats(float_list: List[float]) -> bytes:
    """ Decode a received byte list, into a float list as specified by the rr protocol"""
    data = struct.pack('%sf' % len(float_list), *float_list)
    return data

def pack_protocol_v2_request_data(data):
    """ Re-pack the V2 request data into the correct format """
    packed_data = list()
    for i, x in enumerate(data):
        packed_data.append(x & 0xFF) 
        packed_data.append(x >> 8 & 0x7F)
    return packed_data

def encode_single_packet(device_id: int, packet_id: int, data: Union[List[float], List[int]]) -> bytes:
    """ 
    Encode a single packet based on it's type. 
        - Packet ID's > 0xFF will be encoded as a protocol V2 packet. Protocol V2 must be enabled to parse this packet. 
        - Packet ID's <= 0xFF will be encoded as a (standard) protocol V1 packet. 
    """
    if (packet_id == PacketID.REQUEST) and any(isinstance(data[i], float) for i in range(len(data))):
        raise ValueError("Request must be List of integer packet ID's")
    
    is_v2_packet = (packet_id > 0xFF)
    is_v2_request = (packet_id == PacketID.REQUEST) and any(data[i] > 0xFF for i in range(len(data))) 
    

    if is_v2_request:
        data = pack_protocol_v2_request_data(data)
        option = PacketOption.PROTOCOL_V2 | PacketOption.FWD_ALLOW | PacketOption.IS_DEMAND
    elif is_v2_packet:
        option = PacketOption.PROTOCOL_V2 | PacketOption.FWD_ALLOW | PacketOption.IS_DEMAND
    else:
        option = None

    if PacketID.PacketType[packet_id] and PacketID.PacketType[packet_id] == float:
        return encode_packet(device_id, packet_id, encode_floats(data), option)  # type: ignore check is sufficient 
    else:
        return encode_packet(device_id, packet_id, data, option)  # type: ignore check is sufficient 
    
def create_serial_connection(serial_port: str, BAUD_RATE:int=115200) -> serial.Serial:
    return serial.Serial(serial_port, 
                         baudrate=BAUD_RATE, 
                         stopbits=serial.STOPBITS_ONE,
                         parity=serial.PARITY_NONE, 
                         bytesize=serial.EIGHTBITS, 
                         timeout=0.1)

def create_socket_connection() -> socket.socket:
    _socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    _socket.setblocking(False)
    return _socket


class PacketReader:
    """
    Packet Reader
    Helper class to read bytes, account for the possibility of incomplete packets on arrival.

    Usage:
        packet_reader = PacketReader()
        data == .... receive data here as bytes ...
        packets = packet_reader.receive_bytes()
    """
    incomplete_packets = b''

    def receive_bytes(self, data: bytes) -> List[Tuple[int, int, bytes, Optional[None]]]:
        ''' Receive data, and return a decoded packet '''
        packet_list = []
        encoded_packets, self.incomplete_packets = packet_splitter(self.incomplete_packets + data)
        if not encoded_packets:
            return packet_list
        
        for encoded_packet in encoded_packets:
            if not encoded_packet:
                continue
            
            decoded_packet = parse_packet(encoded_packet)
            packet_list.append(decoded_packet)
        
        return packet_list

class RSProtocol:
    
    def __init__(self, connection: Union[serial.Serial, socket.socket], address:tuple=("0.0.0.0", 0), half_duplex:bool=False) -> None:
        self.packet_reader = PacketReader()
        self.connection: Union[serial.Serial, socket.socket] = connection
        self.address = address
        self.callbacks = dict()
        self.half_duplex = half_duplex
        self.half_duplex_timer = time.perf_counter()
        self._bytes_buffer = b''

    def close_connection(self):
        pass

    def _read_buff(self) -> bytes:
        _bytes_buffer = b''
        
        try:
            if isinstance(self.connection, serial.Serial):
                return self.connection.read(self.connection.in_waiting)
            elif isinstance(self.connection, socket.socket):
                recv_bytes = self.connection.recv(4096)
                _bytes_buffer = recv_bytes
                while recv_bytes != b'':
                    recv_bytes = self.connection.recv(4096)
                    _bytes_buffer += recv_bytes
                return _bytes_buffer
            else:
                raise ValueError("Invalid connection type")
        except:
            return _bytes_buffer

    def half_duplex_locked(self):
        status = False
        if self.half_duplex:
            status = time.perf_counter() < self.half_duplex_timer
        return status
    
    def _write_buff(self, packet: bytes):
        self._bytes_buffer += packet
        if self.half_duplex_locked():
            return
        
        if isinstance(self.connection, serial.Serial):
            self.connection.write(self._bytes_buffer)
            time.sleep(0.0001)  # HACK: back-to-back write calls will fail without a sleep
        elif isinstance(self.connection, socket.socket):
            self.connection.sendto(self._bytes_buffer, self.address)
        else:
            raise ValueError("Invalid connection type")
        
        self._bytes_buffer = b''
        self.half_duplex_timer = time.perf_counter() + HALF_DUPLEX_TIMEOUT        
    
    def write(self, device_id: int, packet_id: int, data: Union[List[float], List[int], float, int]):
        """ Write packet to the connected device """
        if not isinstance(data, list):
            data = [data]

        packet = encode_single_packet(device_id, packet_id, data)
        self._write_buff(packet)
    
    def read_raw(self) -> List[Tuple[int, int, bytes, Optional[None]]]:
        """ Read raw packets from the connected device """
        read_data = self._read_buff()

        if not read_data:
            return list()

        return self.packet_reader.receive_bytes(read_data)
    
    def read(self):
        """ Read decoded packets from the connected device """
        raw_packets = self.read_raw()
        if not raw_packets:
            return list()

        decoded_packets = list()
        for packet in raw_packets:
            read_device_id, read_packet_id, data_bytes, options = packet

            if read_packet_id not in PacketID.PacketType:
                logger.info(f" Recieved unknown packet type: {hex(read_packet_id)}. Decoding packet as integers.")
                data = decode_ints(data_bytes)

            packet_type = PacketID.PacketType.get(read_packet_id)
            if packet_type == float:
                data = decode_floats(data_bytes)
            elif packet_type == int:
                data = decode_ints(data_bytes)
            
            self.run_callback(read_device_id, read_packet_id, data)    
            decoded_packets.append([read_device_id, read_packet_id, data])
        
        return decoded_packets

    def find_request_packet(self, packets, device_id, packet_id) -> Union[List[int], List[float]]:
        """ 
        Finds the requested packet from a list of packets.
        NOTE: All other pending packets will be discarded when this method is called.
        """
        for packet in packets:
            read_device_id, read_packet_id, data_bytes, _ = packet
            if read_device_id != device_id:
                continue
            if read_packet_id != packet_id:
                continue
                
            if read_packet_id not in PacketID.PacketType:
                logger.info(f" Recieved unknown packet type: {hex(read_packet_id)}. Decoding packet as integers.")
                return decode_ints(data_bytes)

            packet_type = PacketID.PacketType.get(read_packet_id)
            if packet_type == float:
                return decode_floats(data_bytes)
            elif packet_type == int:
                return decode_ints(data_bytes)
        
        return list()    
    
    
    def request(self, device_id: int, packet_id: int, read_attempts=25, write_attempts=2, timeout=RESPONSE_TIMEOUT) -> Union[List[int], List[float]]:
        """ Request a single packet from the connected device and return the result """
        
        for _ in range(write_attempts):
            self.write(device_id, PacketID.REQUEST, [packet_id])
            for _ in range(read_attempts):
                time.sleep(timeout)
                packets = self.find_request_packet(self.read_raw(), device_id, packet_id)
                if packets:
                    return packets
                    
        return list()
    
    def is_close(self, data_1: List, data_2: List, tol=0.01) -> Tuple[bool, List]:
        """ Check if two lists of data are equal """
        if len(data_1) != len(data_2):
            return False, data_2
            
        if len(data_1) != len(data_2) or any(abs(a - b) > tol for a, b in zip(data_1, data_2)):
            return False, data_2
        
        return True, data_2
    
    def set(self, device_id: int, packet_id: int, data: Union[List[float], List[int], float, int]):
        ''' Set value on the connected device and verify the recieved data againt the set value '''
        data = data if isinstance(data, list) else [data]
        
        self.write(device_id, packet_id, data)
        recv_data = self.request(device_id, packet_id)
        
        return self.is_close(data, recv_data)
    
    def save(self, device_id: int) -> bool:
        """Save the current configuration to the device's settings."""
        self.set(device_id, PacketID.MODE, [0x08]) 
        
        initial_save_packet = self.request(device_id, PacketID.SAVE)
        if initial_save_packet is None:
            self.set(device_id, PacketID.MODE, [Mode.STANDBY]) 
            logger.info(" Failed to get initial save response.")
            return False

        self.write(device_id, PacketID.SAVE, [0])
        save_packet = self.request(device_id, PacketID.SAVE)
        if save_packet is None or save_packet[0] < initial_save_packet[0] + 1:
            self.set(device_id, PacketID.MODE, [Mode.STANDBY]) 
            logger.info(" Failed to get final save response.")
            return False
        
        self.set(device_id, PacketID.MODE, [Mode.STANDBY]) 
        return True
    
    def register_callback(self, device_id, packet_id, callback):
        """ Register a packet callback """
        if device_id not in self.callbacks:
            self.callbacks[device_id] = {}
        if packet_id not in self.callbacks[device_id]:
            self.callbacks[device_id][packet_id] = []
        self.callbacks[device_id][packet_id].append(callback)

    def remove_callback(self, device_id, packet_id, callback=None):
        """ Remove a packet callback """
        if device_id in self.callbacks and packet_id in self.callbacks[device_id]:
            if callback:
                if callback in self.callbacks[device_id][packet_id]:
                    self.callbacks[device_id][packet_id].remove(callback)
                if not self.callbacks[device_id][packet_id]:  # Remove the packet_id if no callbacks remain
                    del self.callbacks[device_id][packet_id]
            else:
                del self.callbacks[device_id][packet_id]  # Remove all callbacks for the packet_id
            if not self.callbacks[device_id]:  # Remove the device_id if no packet_ids remain
                del self.callbacks[device_id]

    def run_callback(self, device_id, packet_id, data):
        """ Run the packet callbacks """
        callbacks = self.callbacks.get(device_id, {}).get(packet_id, [])
        if callbacks:
            for callback in callbacks:
                callback(device_id, packet_id, data)
        else:
            logger.debug(f" No callbacks registered for device_id={device_id}, packet_id={packet_id}")