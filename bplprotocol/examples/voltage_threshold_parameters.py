import time
from bplprotocol import BPLProtocol, PacketID, PacketReader
import socket

MANIPULATOR_IP_ADDRESS = "192.168.2.4"
MANIPULATOR_PORT = 6789

if __name__ == '__main__':

    manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0)

    enabled = 1.0
    v_min = 20.0  # V
    v_max = 27.0  # V
    thresh_time = 3.0  # seconds

    # To Send the voltage threshold parameters to 708 (0x0D)

    voltage_threshold_packet = \
        BPLProtocol.encode_packet(0x0D, PacketID.VOLTAGE_THRESHOLD_PARAMETERS, BPLProtocol.encode_floats([enabled, v_min, v_max, thresh_time]))

    sock.sendto(voltage_threshold_packet, manipulator_address)

    time.sleep(0.5)

    # Save the parameters
    # Set the device to factory Mode
    set_factory_mode = BPLProtocol.encode_packet(0x0D, PacketID.MODE, bytes([8]))
    sock.sendto(set_factory_mode, manipulator_address)

    save_packet = BPLProtocol.encode_packet(0x0D, PacketID.SAVE, bytes([0]))
    sock.sendto(save_packet, manipulator_address)

    time.sleep(1.5)
    set_standby_mode = BPLProtocol.encode_packet(0x0D, PacketID.MODE, bytes([0]))
    sock.sendto(set_standby_mode, manipulator_address)


