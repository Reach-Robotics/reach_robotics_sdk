
from bplprotocol import BPLProtocol, PacketID, PacketReader


# install pyserial with pip install pyserial

import time

import serial


if __name__ == '__main__':

    device_ids = [0x01, 0x02, 0x03, 0x04, 0x05]

    frequency = 20

    packet_reader = PacketReader()

    serial_port_name = "COM10"

    serial_port = serial.Serial(serial_port_name, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)

    request_packet = b''

    # Request packets can be concatenated
    for device_id in device_ids:
        request_packet += BPLProtocol.encode_packet(device_id, PacketID.REQUEST, bytes(PacketID.POSITION))

    while True:

        # Send request packet
        serial_port.write(request_packet)

        position_responses = {}
        # Read request packets

        start_time = time.time()
        while time.time() < start_time + 1/frequency:
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
                        if read_device_id in device_ids and read_packet_id == PacketID.POSITION:

                            position = BPLProtocol.decode_floats(data_bytes)[0]

                            position_responses[read_device_id] = position
        print(f"Positions {position_responses}")
