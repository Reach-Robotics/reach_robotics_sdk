import time
import argparse
import logging
import math

from device.manipulator import Manipulator
from rs_protocol import RSProtocol, create_socket_connection, create_serial_connection


logging.basicConfig(level=logging.ERROR)

STARTUP_MESSAGE = (
    "\n"
    "\n================================================================================"
    "\nReach Robotics Kinematic Control Application"
    "\n================================================================================"
    "\n"
    "\nAn example of inverse kinematic position control of a Reach Robotics 5FN/7FN"
    "\nmanipulator. "
    "\n"
    "\nThis application commands the manipulator to move the end-effector in a circle "
    "\nin the cartisian X-Y plane starting from the initial pose." 
    "\n"
    "\nWARNING: If the commanded positions are outside the manipulator's reachable "
    "\nworkspace, the manipulator may move unexpectedly on the next valid command." \
    "\nIt is the users responsibility to ensure all commanded positions are safe."
    "\n"
    "\nNotes: "
    "\n  [1] This application is supported by Reach Robotics products with 5 or more "
    "\n      functions. For products with fewer than 5 functions, kinematics "
    "\n      requests/demands are not supported. Additionally, for products with less "
    "\n      than 7FN, orbit controls will be ignored."
    "\n  [2] Inverse kinematics must be enabled on the manipulator to use this app. To"
    "\n      check if inverse kinematics is enabled, email support@reachrobotics.com."
    "\n"
    "\nUsage:"
    "\n  Serial (RS-232): "
    "\n     python3 ./examples/kinematic_position_control.py -c serial -sp COM8"
    "\n  Serial (RS-485): "
    "\n     python3 ./examples/kinematic_position_control.py -c serial -sp COM8 --half_duplex"
    "\n  Ethernet (UDP):  "
    "\n     python3 ./examples/kinematic_position_control.py -c udp -ip 192.168.1.5 -up 6789"
    "\n"
    "\nDISCLAIMER: this application is for demonstrations purposes only and is not "
    "\nintended to be a complete solution."
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


def main() -> None:
    print(STARTUP_MESSAGE)
    
    rs_protocol = get_rs_protocol_connection()
    manipulator = Manipulator(rs_protocol)

    manipulator.update()
    time.sleep(0.05)  # Allow time for initial updates
    manipulator.update()

    T_center = manipulator.get_end_effector_transform()

    # Draw a 2D circle in the global XY plane with 100 mm diameter (0.1 m).
    radius_m = 0.05
    num_points = 360
    dt = 0.01

    # Offset the circle center so the first point (theta=0) matches current pose.
    center_x = T_center[0, 3] - radius_m
    center_y = T_center[1, 3]

    for i in range(num_points + 1):
        theta = (2.0 * math.pi * i) / num_points

        T_target = T_center.copy()
        T_target[0, 3] = center_x + radius_m * math.cos(theta)
        T_target[1, 3] = center_y + radius_m * math.sin(theta)

        manipulator.ee_position_global(T_target)
        manipulator.update()
        time.sleep(dt)


if __name__ == '__main__':
    main()
