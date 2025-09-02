import logging
import argparse
from dataclasses import dataclass
from rs_protocol import RSProtocol, PacketID, create_socket_connection, create_serial_connection


logging.basicConfig(level=logging.ERROR)


DEVICE_ID = 0x01
FREQUENCY = 10  # Hz

STARTUP_MESSAGE = (
    "\n"
    "\n================================================================================"
    "\nReach Robotics Telemetry Heartbeat Example"
    "\n================================================================================"
    "\n"
    "\nThis application demonstrates how to receive telemetry at a fixed heartbeat "
    "\ninterval using either serial or UDP (Ethernet) communication. Please ensure your"
    "\nmanipulator is connected and powered on before starting."
    "\n"
    "\nNOTE: "
    "\n    [1] Heartbeats can not be stopped and restarted on Reach Robotics devices."
    "\n    [2] Maximum heartbeat frequency is 255 Hz, however, actual frequency is "
    "\n        limited by connection bandwidth."
    "\n"
    "\nUsage:"
    "\n  Serial (RS-232): "
    "\n    python3 ./examples/telemetry_heartbeat.py -c serial -sp COM8"
    "\n  Serial (RS-485): "
    "\n    python3 ./examples/telemetry_heartbeat.py -c serial -sp COM8 --half_duplex"
    "\n  Ethernet (UDP):  "
    "\n    python3 ./examples/telemetry_heartbeat.py -c udp -ip 192.168.1.5 -up 6789"
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


def set_heartbeat(rs_protocol: RSProtocol, device_id: int, frequency_hz: int):
    """ Set the heartbeat interval and packets for the device. """
    if frequency_hz < 0 or frequency_hz > 255:
        raise ValueError("Frequency must be a positive integer between 0-255.")

    rs_protocol.write(device_id, PacketID.HEARTBEAT_FREQUENCY_HZ, frequency_hz)
    rs_protocol.write(device_id, PacketID.HEARTBEAT_PACKETS, [PacketID.POSITION, PacketID.VELOCITY, PacketID.CURRENT])

@dataclass
class Telemetry:
    q: float = 0.0
    dq: float = 0.0
    Iq: float = 0.0

    def position_callback(self, device_id, packet_id, data):
        self.q = data[0]

    def velocity_callback(self, device_id, packet_id, data):
        self.dq = data[0]

    def current_callback(self, device_id, packet_id, data):
        self.Iq = data[0]
        

def monitor_telemetry(rs_protocol: RSProtocol, telemetry: Telemetry):
    print(f"--- Telemetry of device 0x0{DEVICE_ID} at {FREQUENCY} Hz ---")

    # Callbacks are used to simplify the data parsing. When a packet is recieved by the protocol
    # the corresponding callback function is called with the parsed data and stored.
    rs_protocol.register_callback(DEVICE_ID, PacketID.POSITION, telemetry.position_callback)
    rs_protocol.register_callback(DEVICE_ID, PacketID.VELOCITY, telemetry.velocity_callback)
    rs_protocol.register_callback(DEVICE_ID, PacketID.CURRENT, telemetry.current_callback)

    while True:
        # Update telemetry via callbacks. If a packet is recieved that matches the callback 
        # fingerprint then the callback will be triggered and the telemetry data updated. 
        rs_protocol.read()
        print(f"\rPosition: {telemetry.q:6.1f} rad, "
              f"Velocity: {telemetry.dq:6.1f} rad/s, "
              f"Current: {telemetry.Iq:6.1f} mA  \033[K", 
              end='', 
              flush=True)


def main() -> None:
    print(STARTUP_MESSAGE)
    rs_protocol = get_rs_protocol_connection()
    telemetry = Telemetry()

    set_heartbeat(rs_protocol, DEVICE_ID, FREQUENCY)
    monitor_telemetry(rs_protocol, telemetry)


if __name__ == '__main__':
    main()