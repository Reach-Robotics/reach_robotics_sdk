import logging
import argparse
from rs_protocol import RSProtocol, PacketID, Mode, create_socket_connection, create_serial_connection


logging.basicConfig(level=logging.ERROR)

STARTUP_MESSAGE = (
    "\n"
    "\n================================================================================"
    "\nReach Robotics Request Jaw Calibration Example"
    "\n================================================================================"
    "\n"
    "\nAn example application for requesting a jaw calibration on a Reach Robotics "
    "\nmanipulator"
    "\n"
    "\nUsage:"
    "\n  Serial (RS-232): "
    "\n    python3 ./examples/request_jaw_calibration.py -c serial -sp COM8"
    "\n  Serial (RS-485): "
    "\n    python3 ./examples/request_jaw_calibration.py -c serial -sp COM8 --half_duplex"
    "\n  Ethernet (UDP):  "
    "\n    python3 ./examples/request_jaw_calibration.py -c udp -ip 192.168.1.5 -up 6789"
    "\n"
    "\nDISCLAIMER: this application is for demonstration purposes only and is not intended"
    "\nto be a complete solution.\n"
)


def get_rs_protocol_connection() -> RSProtocol:
    parser = argparse.ArgumentParser(description="Select connection type and parameters.")
    parser.add_argument('-c', '--connection', choices=['serial', 'udp'], default='udp',
                        help="Connection type: 'serial' for RS-232/RS-485, 'udp' for Ethernet (default: udp)")
    parser.add_argument('-sp', '--serial_port', type=str, default="COM15",
                        help="Serial port to use for serial connection (default: COM15)")
    parser.add_argument('--half_duplex', action='store_true',
                        help="Enable half-duplex mode for RS-485 communication")
    parser.add_argument('-ip', '--ip_address', type=str, default="192.168.1.5",
                        help="IP address for UDP connection (default: 192.168.1.5)")
    parser.add_argument('-up', '--udp_port', type=int, default=6789,
                        help="UDP port for Ethernet connection (default: 6789)")
    args = parser.parse_args()

    if args.connection == 'serial':
        return RSProtocol(create_serial_connection(args.serial_port), half_duplex=args.half_duplex)
    else:
        return RSProtocol(create_socket_connection(), (args.ip_address, args.udp_port))


def request_jaw_calibration(rs_protocol: RSProtocol):
    default_current = -1000  # mA (NOTE: this value can be increase in the case that friction is too high)
    default_velocity = 50  # rad/s

    print(f"--- Request Jaw Calibration Example ---"
          f"\nThe Jaws will move at maximum {default_current} mA and {default_velocity} rad/s until the jaws are closed. \n"
          f"NOTE: Velocity demand in this case is the rotor speed. The velocity output with be default_velocity * 1 / gear_ratio.\n")
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


def main() -> None:
    print(STARTUP_MESSAGE)
    rs_protocol = get_rs_protocol_connection()
    request_jaw_calibration(rs_protocol)
    monitor_status(rs_protocol)


if __name__ == '__main__':
    main()