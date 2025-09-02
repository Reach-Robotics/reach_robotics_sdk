import logging
import socket
from rs_protocol import RSProtocol, PacketID, create_socket_connection, create_serial_connection


logging.basicConfig(level=logging.ERROR)

SECTION_BREAK = "\n================================================================================"
STARTUP_MESSAGE = (
    "\n"
    "\n================================================================================"
    "\nReach Robotics Debug Application"
    "\n================================================================================"
    "\n"
    "\nThis application provides an interface to debug connections to reach robotics "
    "\ndevices."
    "\n"
    "\nUsage:"
    "\n    python3 ./examples/debug.py"
    "\n"
)


class TextPrompt:
    def __init__(self, message, validate=None):
        self.message = message
        self.validate = validate

    def ask(self):
        while True:
            val = input(self.message + ' ')
            if self.validate:
                valid = self.validate(val)
                if valid is True:
                    return val
                elif isinstance(valid, str):
                    print(valid)
                else:
                    print("Invalid input. Please try again.")
            else:
                return val


class SelectPrompt:
    def __init__(self, message, choices):
        self.message = message
        self.choices = choices

    def ask(self):
        while True:
            print(self.message)
            for idx, choice in enumerate(self.choices, 1):
                print(f"  {idx}. {choice}")
            val = input(f"Enter a number (1-{len(self.choices)}): ").strip()
            if val.isdigit():
                idx = int(val)
                if 1 <= idx <= len(self.choices):
                    return self.choices[idx - 1]
            print("Invalid selection. Please try again.\n")


class DebugApp:
    connection = None

    def __init__(self) -> None:
        self.state = "choose_connection"
        self.menu_methods = {
            "choose_connection": self.choose_connection,
            "UDP": self.setup_udp,
            "RS-485": self.setup_rs485,
            "RS-232": self.setup_rs232,
            "ping udp": self.ping_udp
        }
    
        self.ip = "0.0.0.0"
        self.port = 6789
        self.serial_port = ""
        self.half_duplex = False

    def run(self):
        print(STARTUP_MESSAGE)
        print(SECTION_BREAK)

        while True:
            self.menu_methods[self.state]() 

    def choose_connection(self):
        self.state = SelectPrompt(
            "Select connection type:",
            ["UDP", "RS-485", "RS-232"]
        ).ask()
    
    def setup_rs485(self):
        print("Enter the serial port:")
        print("  Windows example: COM3")
        print("  Linux example: /dev/ttyUSB0")
        self.serial_port = TextPrompt("Serial port:").ask()

    def setup_rs232(self):
        print("Enter the serial port:")
        print("  Windows example: COM3")
        print("  Linux example: /dev/ttyUSB0")
        self.serial_port = TextPrompt("Serial port:").ask()

    def setup_udp(self):
        def validate_ip(val):
            parts = val.strip().split('.')
            if len(parts) != 4:
                return "IP address must have four octets."
            for part in parts:
                if not part.isdigit() or not 0 <= int(part) <= 255:
                    return "Each octet must be a number between 0 and 255."
            return True

        self.ip = TextPrompt(
            "Enter the IP address (e.g., 192.168.2.3):",
            validate=validate_ip
        ).ask()
        self.port = TextPrompt(
            "Enter the UDP port (e.g., 6789):",
            validate=lambda val: True if val.isdigit() else "Port must be a number"
        ).ask()

        self.state = "ping udp"

    def ping_udp(self):
        print(f"Pinging UDP {self.ip}:{self.port}...")
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(2)  # 2 second timeout
        try:
            message = b'ping'
            sock.sendto(message, (self.ip, int(self.port)))
            data, addr = sock.recvfrom(1024)
            print(f"Received response from {addr}: {data}")
        except socket.timeout:
            print("No response received (timeout).")
        finally:
            sock.close()
        self.state = "choose_connection"


if __name__ == '__main__':
    # RSProtocol(create_socket_connection(), (args.ip_address, args.udp_port))
    debug_app = DebugApp()
    debug_app.run()