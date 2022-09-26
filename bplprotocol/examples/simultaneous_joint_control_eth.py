from bplprotocol import BPLProtocol, PacketID
import time

import socket

if __name__ == '__main__':

    MANIPULATOR_IP_ADDRESS = '192.168.2.4'
    MANIPULATOR_PORT = 6789
    manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Desired position from end effector to base A -> G
    desired_positions = [10.0, 0.5, 1.5707, 1.5707, 1.5707, 2.8, 3.14159]

    packets = b''
    for index, position in enumerate(desired_positions):
        device_id = index + 1
        packets += BPLProtocol.encode_packet(device_id, PacketID.POSITION, BPLProtocol.encode_floats([position]))

    # Send joints to desired_positions
    sock.sendto(packets, manipulator_address)

