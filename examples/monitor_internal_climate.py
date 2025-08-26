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

def main():
    rs_protocol = get_rs_protocol_connection()
    monitor_internal_climate(rs_protocol)
         

if __name__ == '__main__':
    main()