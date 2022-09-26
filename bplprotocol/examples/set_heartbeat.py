"""bplprotocol/examples/set_heartbeat.py"""
import time

from bplprotocol import BPLProtocol, PacketID, PacketReader

import socket

MANIPULATOR_IP_ADDRESS = "192.168.2.4"
MANIPULATOR_PORT = 6789

HEARTBEAT_PACKETS = [PacketID.POSITION, PacketID.VELOCITY, 0, 0, 0, 0, 0, 0, 0, 0]

DEVICE_IDS = [1, 2, 3, 4, 5, 6, 7]

FREQUENCY = 10

if __name__ == '__main__':

    manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    sock.settimeout(0)

    packet_to_send = b''

    for device_id in DEVICE_IDS:

        heartbeat_packets_set = BPLProtocol.encode_packet(device_id, PacketID.HEARTBEAT_SET, bytes(HEARTBEAT_PACKETS))
        sock.sendto(heartbeat_packets_set, manipulator_address)

        heartbeat_frequency_set = BPLProtocol.encode_packet(device_id, PacketID.HEARTBEAT_FREQUENCY, bytes([FREQUENCY]))
        sock.sendto(heartbeat_frequency_set, manipulator_address)

        print(f"Heartbeat set for Device 0x{device_id:X}")


    # Read the corresponding packets from the TX2.

    positions = ["-"] * len(DEVICE_IDS)
    pr = PacketReader()
    while True:
        try:
            recv_bytes, address = sock.recvfrom(4096)
        except BaseException:
            recv_bytes = b''

        if recv_bytes:
            packets = pr.receive_bytes(recv_bytes)

            for device_id, packet_id, data in packets:
                if packet_id == PacketID.POSITION:
                    position = BPLProtocol.decode_floats(data)[0]

                    idx = DEVICE_IDS.index(device_id)

                    positions[idx] = f"{position:.2f}"

                    print_string = f"{time.perf_counter():.3f}| Positions: "
                    for dev_id, position in zip(DEVICE_IDS, positions):
                        print_string += f"{dev_id}: {position}, "
                    print(print_string)
                    # time.sleep(0.001)

                    # print(f"\r0x{device_id:X} Position: {position:.2f}", end="", flush=True)

