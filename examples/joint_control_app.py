import sys
import argparse
import logging
from PyQt5 import QtWidgets

from device.manipulator import Manipulator
from ui.joint_control_ui import JointControlUI
from rs_protocol import RSProtocol, create_socket_connection, create_serial_connection

logging.basicConfig(level=logging.ERROR)

STARTUP_MESSAGE = (
    "\n\n============================================\n"
    " Reach Robotics Joint Control Application\n"
    "============================================\n\n"

    "An example application for joint-level control of a Reach Robotics manipulator.\n\n"
    "This application provides a graphical interface for controlling and monitoring\n"
    "Reach Robotics manipulators using either serial or UDP (Ethernet) communication.\n"
    "Please ensure your manipulator is connected and powered on before starting.\n\n"

    "Usage:\n"
    "  7FN Serial (RS-232): python3 ./examples/joint_control_app.py -c serial -sp COM8 -n 7\n"
    "  7FN Serial (RS-485): python3 ./examples/joint_control_app.py -c serial -sp COM8 --half_duplex -n 7\n"
    "  7FN Ethernet (UDP):  python3 ./examples/joint_control_app.py -c udp -ip 192.168.1.5 -up 6789 -n 7\n\n"

    "DISCLAIMER: this application is for demonstration purposes only and is not intended\n"
    "to be a complete solution.\n"
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
    parser.add_argument('-n', '--num_actuators', type=int, default=7,
                        help="Number of actuators (default: 7)")
    args = parser.parse_args()

    if args.connection == 'serial':
        return RSProtocol(create_serial_connection(args.serial_port), half_duplex=args.half_duplex)
    else:
        return RSProtocol(create_socket_connection(), (args.ip_address, args.udp_port))

def main() -> None:
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('-n', '--num_actuators', type=int, default=7)
    args, _ = parser.parse_known_args()

    rs_protocol = get_rs_protocol_connection()
    manipulator = Manipulator(rs_protocol, num_actuators=args.num_actuators)

    app = QtWidgets.QApplication(sys.argv)
    visualizer = JointControlUI(manipulator)
    visualizer.show()

    print(STARTUP_MESSAGE)

    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
