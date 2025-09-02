import sys
import argparse
import logging
from PyQt5 import QtWidgets

from device.manipulator import Manipulator
from ui.kinemaitc_control_ui import KinematicControlUI
from rs_protocol import RSProtocol, create_socket_connection, create_serial_connection


logging.basicConfig(level=logging.ERROR)

STARTUP_MESSAGE = (
    "\n"
    "\n================================================================================"
    "\nReach Robotics Kinematic Control Application"
    "\n================================================================================"
    "\n"
    "\nAn example application for kinematic control of a Reach Robotics 5FN/7FN"
    "\nmanipulator. "
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
    "\n     python3 ./examples/kinematic_control_app.py -c serial -sp COM8"
    "\n  Serial (RS-485): "
    "\n     python3 ./examples/kinematic_control_app.py -c serial -sp COM8 --half_duplex"
    "\n  Ethernet (UDP):  "
    "\n     python3 ./examples/kinematic_control_app.py -c udp -ip 192.168.1.5 -up 6789"
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
    rs_protocol = get_rs_protocol_connection()
    manipulator = Manipulator(rs_protocol)

    app = QtWidgets.QApplication(sys.argv)
    visualizer = KinematicControlUI(manipulator)
    visualizer.show()

    print(STARTUP_MESSAGE)

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
