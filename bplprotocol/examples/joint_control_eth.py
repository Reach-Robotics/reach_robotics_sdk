

from bplprotocol import BPLProtocol, PacketID
import time

import socket

if __name__ == '__main__':

    device_id = 0x02  # Joint B

    MANIPULATOR_IP_ADDRESS = '192.168.2.4'
    MANIPULATOR_PORT = 6789
    manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Send to a position of 0.5 radians
    sock.sendto(BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([0.5])), manipulator_address)

    time.sleep(5)

    # Send a velocity command of 0.3
    sock.sendto(BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([0.3])), manipulator_address)

    time.sleep(3)
    # Send a velocity of 0
    sock.sendto(BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([0.0])), manipulator_address)

    time.sleep(3)
    # Send a relative position command of 0
    sock.sendto(BPLProtocol.encode_packet(device_id, PacketID.RELATIVE_POSITION, BPLProtocol.encode_floats([-0.5])), manipulator_address)

    time.sleep(3)
