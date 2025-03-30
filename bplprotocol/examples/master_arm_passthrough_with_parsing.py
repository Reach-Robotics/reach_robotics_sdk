"""
The purpose of this script is to directly passthrough commands from the master arm to an arm
connected to the computer. This script will also parse the data from the master arm and store
the position and velocity of the master arm in a dictionary.

There are several data types returned by the MasterArm,
    - INDEXED_POSITION: joint position relative to an index. The index is set when the device is "un-paused"
    - POSITION_VELOCITY: joint (indexed) position and velocity in a single packet
    - VELOCITY: joint velocity. 

Note 1: 0xC1 only reports velocity using PacketID.VELOCITY. It's values is updated by the handles joystick x axis.
Note 2: 0xC2 reports VELOCITY if the controller is in joystick mode.
Note 3: Joints will report either INDEXED_POSITION or POSITION_VELOCITY depending on Master Arm internal settings.
Note 4: 0xC6 and 0xC7 will only be updated for 7 function MasterArms.
"""
import time
import serial # install pyserial with pip install pyserial
from bplprotocol import BPLProtocol, PacketID, PacketReader
from typing import Optional

POSITION_VELOCITY_DEMAND = 0x0C  # Not available in bplprotocol -> PacketID 

MASTER_ARM_PORT = "COM14"
MANIPULATOR_PORT = "COM2"

position_velocity_dict: dict = {
    0xC1: (0.0, 0.0),  # Controls Device ID: 0x01 - (Linear)
    0xC2: (0.0, 0.0),  # Controls Device ID: 0x02 - (Wrist)
    0xC3: (0.0, 0.0),  # Controls Device ID: 0x03
    0xC4: (0.0, 0.0),  # Controls Device ID: 0x04
    0xC5: (0.0, 0.0),  # Controls Device ID: 0x05
    0xC6: (0.0, 0.0),  # Controls Device ID: 0x06 - (7 function only)
    0xC7: (0.0, 0.0)   # Controls Device ID: 0x07 - (7 function only)
}


def parse_master_arm_data(master_arm: Optional[serial.Serial]) -> bytes:
    if master_arm is None:
        return b''
    
    try:
        data: bytes = master_arm.read(1024)
    except serial.SerialException as e:
        print(f"Error Reading from master arm serial port: {e}")
        data: bytes = b''

    if data != b'':
        packets = packet_reader.receive_bytes(data)
        if packets:
            for packet in packets:
                read_device_id, read_packet_id, data_bytes = packet
                position, velocity = 0.0, 0.0
                
                # Index position velocity mode is typically used for Bravo manipulators 
                if read_packet_id == POSITION_VELOCITY_DEMAND:
                    position = round(BPLProtocol.decode_floats(data_bytes)[0], 2)
                    velocity = round(BPLProtocol.decode_floats(data_bytes)[1], 2)

                # Index position mode is typically used for Alpha manipulators 
                if read_packet_id == PacketID.INDEXED_POSITION:
                    position = round(BPLProtocol.decode_floats(data_bytes)[0], 2)

                # Velocity mode is typically used for the linear and wrist
                if read_packet_id == PacketID.VELOCITY:
                    position = round(BPLProtocol.decode_floats(data_bytes)[0], 2)

                try:
                    position_velocity_dict[read_device_id] = (position, velocity)
                except KeyError as e:
                    print(f"Key {read_device_id} is invalid for position_velocity_dict: {e}")
    
    return data


def forward_master_arm_data(manipulator, data):
    if manipulator and data != b'':
        try:
            manipulator.write(data)
        except BaseException as e:
            print(f"Error Writing to an arms serial port: {e}")


if __name__ == '__main__':
    
    packet_reader: PacketReader = PacketReader()
    
    try:
        master_arm: Optional[serial.Serial] = serial.Serial(MASTER_ARM_PORT, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)
    except serial.SerialException:
        master_arm = None
        print(f"Unable to connect to master arm on port: {MASTER_ARM_PORT}. Please check your connection.")
    
    try:
        manipulator: Optional[serial.Serial] = serial.Serial(MANIPULATOR_PORT, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)
    except serial.SerialException:
        manipulator = None
        print(f"Unable to connect to master arm on port: {MANIPULATOR_PORT}. Please check your connection.")

    while True:
        time.sleep(0.0001)
        data = parse_master_arm_data(master_arm)
        forward_master_arm_data(manipulator, data)
        
        print(f"position_velocity_dict: {position_velocity_dict}")


