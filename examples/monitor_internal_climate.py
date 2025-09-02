import logging
import argparse
from rs_protocol import RSProtocol, PacketID, Mode, create_socket_connection, create_serial_connection


logging.basicConfig(level=logging.ERROR)

STARTUP_MESSAGE = (
    "\n"
    "\n================================================================================"
    "\nReach Robotics Monitor Internal Climate Example"
    "\n================================================================================"
    "\n"
    "\nThis application demonstrates how to monitor internal humidity, temperature, and"
    "\npressure using either serial or UDP (Ethernet) communication. Please ensure your"
    "\nmanipulator is connected and powered on before starting."
    "\n"
    "\nUsage:"
    "\n  Serial (RS-232): "
    "\n    python3 ./examples/monitor_internal_climate.py -c serial -sp COM8"
    "\n  Serial (RS-485): "
    "\n    python3 ./examples/monitor_internal_climate.py -c serial -sp COM8 --half_duplex"
    "\n  Ethernet (UDP):  "
    "\n    python3 ./examples/monitor_internal_climate.py -c udp -ip 192.168.1.5 -up 6789"
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


def monitor_internal_climate(rs_protocol: RSProtocol):
    print(f"--- Monitor Internal Climate ---")
    while True:
        data = rs_protocol.request(0x01, PacketID.INTERNAL_HUMIDITY)
        internal_humidity = data[0] if data else 0.0

        data = rs_protocol.request(0x01, PacketID.INTERNAL_TEMPERATURE)
        internal_temperature = data[0] if data else 0.0

        data = rs_protocol.request(0x01, PacketID.INTERNAL_PRESSURE)
        internal_pressure = data[0] if data else 0.0

        print(f"\rHumidity: {internal_humidity:6.1f} %, Temperature: {internal_temperature:6.1f} °C, Pressure: {internal_pressure:6.1f} Bar  \033[K", end='', flush=True)


def main() -> None:
    print(STARTUP_MESSAGE)
    rs_protocol = get_rs_protocol_connection()
    monitor_internal_climate(rs_protocol)


if __name__ == '__main__':
    main()