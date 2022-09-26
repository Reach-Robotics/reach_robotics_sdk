from bplprotocol import BPLProtocol, PacketID, PacketReader

import time

# install pyserial with pip install pyserial
import serial


if __name__ == '__main__':

    packet_reader = PacketReader()

    serial_port_name = "COM10"

    request_timeout = 0.5  # Seconds

    serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)

    # Requesting Information
    device_id = 0x01  # Jaws

    print(f"Requesting Position from Device {device_id}")

    # Request POSITION from the jaws
    serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.REQUEST, bytes([PacketID.POSITION])))

    start_time = time.time()

    position = None
    while True:
        time.sleep(0.0001)
        try:
            read_data = serial_port.read()
        except BaseException:
            read_data = b''
        if read_data != b'':
            packets = packet_reader.receive_bytes(read_data)
            if packets:
                for packet in packets:
                    read_device_id, read_packet_id, data_bytes = packet
                    if read_device_id == device_id and read_packet_id == PacketID.POSITION:

                        # Decode floats, because position is reported in floats
                        position = BPLProtocol.decode_floats(data_bytes)[0]
                        print(f"Position from Device {device_id} is {position}")

                if position is not None:
                    break

        # Timeout if no response is seen from the device.
        if time.time() - start_time > request_timeout:
            print("Request for Position timed out")
            break

    time.sleep(3)

    print(f"Requesting Software Version from Device {device_id}")
    # Request the Software version from the jaws
    serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.REQUEST, bytes([PacketID.SOFTWARE_VERSION])))
    software_version = None
    start_time = time.time()
    while True:
        time.sleep(0.0001)
        try:
            read_data = serial_port.read()
        except BaseException:
            read_data = b''
        if read_data != b'':
            packets = packet_reader.receive_bytes(read_data)
            if packets:
                for packet in packets:
                    read_device_id, read_packet_id, data_bytes = packet
                    if read_device_id == device_id and read_packet_id == PacketID.SOFTWARE_VERSION:

                        # software version is reported as a list of integers
                        software_version = list(data_bytes)
                        print(f"Software version from Device: {device_id} is {tuple(software_version)}")
                if software_version is not None:
                    break

        # Timeout if no response is seen from the device.
        if time.time() - start_time > request_timeout:
            print("Request for Software version timed out")
            break

    # Requesting multiple packets at once. Position and Velocity. Packets will responding individually from the device
    time.sleep(3)

    print(f"Requesting Position and Velocity from Device {device_id}")

    serial_port.write(BPLProtocol.encode_packet(device_id, PacketID.REQUEST, bytes([PacketID.VELOCITY, PacketID.POSITION])))

    start_time = time.time()
    position = None
    velocity = None
    while True:
        time.sleep(0.0001)
        try:
            read_data = serial_port.read()
        except BaseException:
            read_data = b''
        if read_data != b'':
            packets = packet_reader.receive_bytes(read_data)
            if packets:
                for packet in packets:
                    read_device_id, read_packet_id, data_bytes = packet
                    if read_device_id == device_id and read_packet_id in [PacketID.VELOCITY, PacketID.POSITION]:
                        if read_packet_id == PacketID.POSITION:
                            position = BPLProtocol.decode_floats(data_bytes)[0]
                        elif read_packet_id == PacketID.VELOCITY:
                            velocity = BPLProtocol.decode_floats(data_bytes)[0]

                if position is not None and velocity is not None:
                    break

        # Timeout if no response is seen from the device.
        if time.time() - start_time > request_timeout:
            print("Request for Position and Velocity Timed out")
            break

    if position is not None and velocity is not None:
        print(f"Received Position {position} and Velocity {velocity} from device {device_id}")

