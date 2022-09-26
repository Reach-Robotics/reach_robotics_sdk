import math
from typing import List, Optional

from bplprotocol import BPLProtocol, PacketID, PacketReader

import time

# install pyserial with pip install pyserial
import serial


if __name__ == '__main__':

    packet_reader = PacketReader()

    serial_port_name = "COM19"

    joint_devices = [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
    joint_indexed_positions: List[Optional[float]] = [None] * len(joint_devices)
    joint_velocities: List[Optional[float]] = [None] * len(joint_devices)

    jaw_device_id = 0x01
    jaw_vel_cmd = None
    wrist_device_id = 0x02
    wrist_vel_cmd = None

    mode = "-"

    serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)

    last_print_time = time.time()

    pressed_button = "-"

    print_frequency = 20

    while True:
        time.sleep(0.0001)

        if time.time() - last_print_time > 1/print_frequency:
            last_print_time = time.time()
            print("\n")
            print("Mode:             " + f"\t{mode}")
            print("Devices:            " + "\t".join([f"0X{x:02X}      " for x in joint_devices]))
            print("Idx Pos(deg):     " + "\t".join([f"{x:<7.2f}   " if x is not None else "\t-      " for x in joint_indexed_positions]))
            print("Vel (deg/s):      " + "\t".join([f"{x:<7.2f}   " if x is not None else "\t-      " for x in joint_velocities]))

            print("Jaw CMD(mm/s):    " + (f"\t{jaw_vel_cmd:.2f}"if jaw_vel_cmd is not None else "\t-      "))
            print("Wrist CMD(deg/s): " + (f"\t{wrist_vel_cmd:.2f}" if wrist_vel_cmd is not None else "\t-      "))

        try:
            read_data = serial_port.read(1024)
        except BaseException:
            read_data = b''
        if read_data != b'':
            packets = packet_reader.receive_bytes(read_data)
            if packets:
                for packet in packets:
                    read_device_id, read_packet_id, data_bytes = packet

                    if read_device_id & 0x0F in joint_devices and read_packet_id == 0x0C:
                        idx = joint_devices.index(read_device_id & 0x0F)

                        data_floats = BPLProtocol.decode_floats(data_bytes)
                        if len(data_floats) < 2:
                            continue

                        indexed_position = data_floats[0]
                        velocity = data_floats[1]
                        joint_indexed_positions[idx] = math.degrees(indexed_position)
                        joint_velocities[idx] = math.degrees(velocity)
                    if read_device_id == 0xCE and read_packet_id == 0x01:
                        _mode = list(data_bytes)[0]

                        if _mode == 0x1D:
                            mode = "PAUSED"
                            joint_indexed_positions: List[Optional[float]] = [None] * len(joint_devices)
                            joint_velocities: List[Optional[float]] = [None] * len(joint_devices)
                            pressed_button = "-"

                        elif _mode == 20:
                            mode = "BUTTON_PRESSED: \t" + pressed_button
                            joint_indexed_positions: List[Optional[float]] = [None] * len(joint_devices)
                            joint_velocities: List[Optional[float]] = [None] * len(joint_devices)

                        else:
                            mode = "RUNNING"
                            pressed_button = "-"

                    if read_device_id == 0x0E and read_packet_id == 0x55:
                        if list(data_bytes)[0] == 1:
                            pressed_button = "TOP"
                        elif list(data_bytes)[0] == 0:
                            pressed_button = "BOTTOM"
                        else:
                            pressed_button = "-"

                    if read_device_id & 0x0F == jaw_device_id and read_packet_id == PacketID.VELOCITY:
                        vel = BPLProtocol.decode_floats(data_bytes)
                        jaw_vel_cmd = vel[0]

                    if read_device_id & 0x0F == wrist_device_id and read_packet_id == PacketID.VELOCITY:
                        vel = BPLProtocol.decode_floats(data_bytes)
                        wrist_vel_cmd = math.degrees(vel[0])



