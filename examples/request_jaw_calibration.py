"""
Request Jaw Calibration Example

Description:
This script demonstrates how to request a jaw calibration for a Reach Robotics manipulator.

Supported Products:
- Reach Bravo (All Variants)
- Reach X (All Variants)

"""
import argparse
from rs_protocol import RSProtocol, PacketID, Mode, create_socket_connection, create_serial_connection

import logging
logging.basicConfig()
logging.getLogger().setLevel(logging.ERROR)

def get_rs_protocol_connection():
    parser = argparse.ArgumentParser(description="Select connection type and parameters.")
    parser.add_argument('-c', '--connection', choices=['serial', 'udp'], default='udp')
    parser.add_argument('-sp', '--serial_port', type=str, default="COM15")
    parser.add_argument('--half_duplex', action='store_true')
    parser.add_argument('-ip', '--ip_address', type=str, default="192.168.1.5")
    parser.add_argument('-up', '--udp_port', type=int, default=6789)
    args = parser.parse_args()

    if args.connection == 'serial':
        return RSProtocol(create_serial_connection(args.serial_port), half_duplex=args.half_duplex)
    else:
        return RSProtocol(create_socket_connection(), (args.ip_address, args.udp_port))

def request_jaw_calibration(rs_protocol: RSProtocol):

    default_current = -1000  # mA
    default_velocity = 50  # rad/s

    print(f"--- Request Jaw Calibration Example ---"
          f"\nThe Jaws will move at maximum {default_current} mA and {default_velocity} rad/s until the jaws are closed.\n")
    rs_protocol.write(0x01, PacketID.REQUEST_JAW_CALIBRATION, [default_current, default_velocity])

def monitor_status(rs_protocol: RSProtocol):
    mode = Mode.JAW_CALIBRATION
    while mode == Mode.JAW_CALIBRATION:
        data = rs_protocol.request(0x01, PacketID.VELOCITY)
        velocity = data[0] if data else 0.0
        
        data = rs_protocol.request(0x01, PacketID.CURRENT)
        current = data[0] if data else 0.0
        
        data = rs_protocol.request(0x01, PacketID.MODE)
        mode = data[0] if data else Mode.STANDBY
        
        print(f"\rMode: {Mode(mode).name}, Velocity: {velocity:6.1f} rad/s, Current: {current:7.1f} mA \033[K", end='', flush=True)

    print("\n\nJaw calibration complete.")

def main():
    rs_protocol = get_rs_protocol_connection()
    request_jaw_calibration(rs_protocol)
    monitor_status(rs_protocol)
         

if __name__ == '__main__':
    main()