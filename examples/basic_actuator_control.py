"""
Basic Actuator Control Example

This script demonstrates how to control a Reach Robotics manipulator using
both position and velocity commands through the RS Protocol.

Note:
    - This script assumes a connection to a 7fn manipulator with device ID set (0x01 ... 0x07)
    - In the case of a 5fn manipulator, the values for devices 0x06 and 0x07 will not be update.
"""
import time
from rs_protocol import RSProtocol, PacketID, Mode, create_socket_connection, create_serial_connection

import logging
logging.basicConfig()
logging.getLogger().setLevel(logging.ERROR)

# Device configuration
DEVICE_ID = 0x02
MANIPULATOR_IP_ADDRESS = '192.168.2.3'
MANIPULATOR_UDP_PORT = 6789
MANIPULATOR_SERIAL_PORT = "COM14"

# Control parameters
TARGET_POSITION_RAD = -3.14
VELOCITY_RAD_PER_SEC = 0.3
VELOCITY_COMMAND_INTERVAL_SEC = 0.05  # Send command every 50ms to prevent timeout
VELOCITY_COMMAND_DURATION_SEC = 5.0 
REQUEST_FREQUENCY = 100


def initialise_rs_protocol() -> RSProtocol:
    """Create and return a connection to the device."""
    # Uncomment the appropriate connection method
    # rs_protocol = RSProtocol(
    #     create_socket_connection(), 
    #     (MANIPULATOR_IP_ADDRESS, MANIPULATOR_UDP_PORT)
    # )
    rs_protocol = RSProtocol(
        create_serial_connection(MANIPULATOR_SERIAL_PORT),
        half_duplex=True
    )
    return rs_protocol

def position_control_demo(rs_protocol: RSProtocol):
    print(f"\n--- Position Control Demo (Target: {TARGET_POSITION_RAD} rad) ---")
    rs_protocol.write(DEVICE_ID, PacketID.POSITION, -3.14)
    time.sleep(3)
    print(f"The device mode is: {rs_protocol.request(DEVICE_ID, PacketID.MODE)}")

def velocity_control_demo(rs_protocol: RSProtocol):
    print(f"\n--- Velocity Control Demo ({VELOCITY_RAD_PER_SEC} rad/s) ---")
    print(f"Sending velocity commands for {VELOCITY_COMMAND_DURATION_SEC} seconds...")
    for _ in range(int(VELOCITY_COMMAND_DURATION_SEC / VELOCITY_COMMAND_INTERVAL_SEC)):
        rs_protocol.write(DEVICE_ID, PacketID.VELOCITY, 0.3)
        time.sleep(0.05)
    rs_protocol.write(DEVICE_ID, PacketID.VELOCITY, 0.0)
    print(f"The device mode is: {rs_protocol.request(DEVICE_ID, PacketID.MODE)}")

def standby_mode_change_demo(rs_protocol: RSProtocol):
    print("\n--- Returning to Standby ---")
    rs_protocol.write(DEVICE_ID, PacketID.MODE, [Mode.STANDBY])
    print(f"The device mode is: {rs_protocol.request(DEVICE_ID, PacketID.MODE)}")

def request_feedback_demo(rs_protocol: RSProtocol):
    print("\n--- Requesting positon, velocity, and current ---")
    print(f"Position: {rs_protocol.request(DEVICE_ID, PacketID.POSITION)}")
    print(f"Velocity: {rs_protocol.request(DEVICE_ID, PacketID.VELOCITY)}")
    print(f"Current: {rs_protocol.request(DEVICE_ID, PacketID.CURRENT)}")  

def request_software_version_demo(rs_protocol: RSProtocol):
    print("\n--- Requesting Software Version ---")
    print(f"Software Version: {rs_protocol.request(DEVICE_ID, PacketID.SOFTWARE_VERSION)}")

def read_feedback_loop_demo(rs_protocol: RSProtocol):
        # Read packets continuously 
    print("\n--- Read packets continously ---")
    next_req_time = time.time() + 1 # (1/REQUEST_FREQUENCY)
    position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    current = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    while True:
        packets = rs_protocol.read()
        for packet in packets:
            if packet[1] == PacketID.POSITION:
                position[packet[0]-1] = packet[2][0]
            elif packet[1] == PacketID.VELOCITY:
                velocity[packet[0]-1] = packet[2][0]
            elif packet[1] == PacketID.CURRENT:
                current[packet[0]-1] = packet[2][0]

        if time.time() >= next_req_time:
            next_req_time += (1/REQUEST_FREQUENCY)
            rs_protocol.write(0xFF, PacketID.REQUEST, [PacketID.POSITION, PacketID.VELOCITY, PacketID.CURRENT])
            print(f"Position: {position}")
            # print(f"Velocity: {velocity}")  # Uncomment to print actuator velocities 
            # print(f"Current: {current}")  # Uncomment to print actuator velocities 

def set_heartbeat_demo(rs_protocol: RSProtocol, interval: int):
    """Set the heartbeat interval for the device."""
    if interval < 0 or interval > 255:
        raise ValueError("Interval must be a positive integer between 0-255.")
    
    rs_protocol.write(DEVICE_ID, PacketID.RESET, 0)
    time.sleep(0.5)
    rs_protocol.write(DEVICE_ID, PacketID.HEARTBEAT_FREQUENCY_HZ, interval)
    rs_protocol.write(DEVICE_ID, PacketID.HEARTBEAT_PACKETS, [PacketID.POSITION])

def main():
    rs_protocol = initialise_rs_protocol()

    # set_heartbeat_demo(rs_protocol, 1)
    # position_control_demo(rs_protocol)
    # velocity_control_demo(rs_protocol)
    # standby_mode_change_demo(rs_protocol)
    # request_feedback_demo(rs_protocol)
    # request_software_version_demo(rs_protocol)
    # read_feedback_loop_demo(rs_protocol)
         

if __name__ == '__main__':
    main()