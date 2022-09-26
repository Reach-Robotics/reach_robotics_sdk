"""bplprotocol/examples/ft_sensor.py
Example to tare and read the force and torque.

Taring the FT sensor is done by sending the ATI_FT packet to device 0x0D.

Reading the FT Sensor can be done by requesting ATI_FT packets from device 0x0D
"""
import time
from bplprotocol import BPLProtocol, PacketID, PacketReader
import socket

MANIPULATOR_IP_ADDRESS = "192.168.2.4"
MANIPULATOR_PORT = 6789

if __name__ == '__main__':

    manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0)

    # To TARE THe sensor send an ATI_FT Packet to Device 0x0D

    tare_packet = \
        BPLProtocol.encode_packet(0x0D, PacketID.ATI_FT_READING, BPLProtocol.encode_floats([0., 0., 0., 0., 0., 0.]))

    sock.sendto(tare_packet, manipulator_address)

    print("Tared the FT Sensor")
    time.sleep(0.5)

    # To Read the packers request from the ATI_FT Packet from device 0x0D

    request_timeout = 0.1  # seconds

    pr = PacketReader()

    readings = 0
    while True:
        request_packet = BPLProtocol.encode_packet(0x0D, PacketID.REQUEST, bytes([PacketID.ATI_FT_READING]))

        start_time = time.time()
        sock.sendto(request_packet, manipulator_address)

        packet_received = False
        while time.time() - start_time < request_timeout:
            try:
                recv_bytes, address = sock.recvfrom(4096)

            except BaseException:
                recv_bytes = b''

            if recv_bytes:
                packets = pr.receive_bytes(recv_bytes)

                for device_id, packet_id, data in packets:
                    if packet_id == PacketID.ATI_FT_READING:
                        ft_readings = BPLProtocol.decode_floats(data)

                        print(f"Received FT Reading: FX: {ft_readings[0]}, FY: {ft_readings[1]}, FZ: {ft_readings[2]} "
                              f"TX: {ft_readings[3]}, TY: {ft_readings[4]}, TZ: {ft_readings[5]}")

                        readings += 1
                        packet_received = True

                if packet_received:
                    break

