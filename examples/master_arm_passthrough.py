import sys
import argparse
import logging
import time
import serial

logging.basicConfig(level=logging.ERROR)

STARTUP_MESSAGE = (
    "\n"
    "\n================================================================================"
    "\n Reach Robotics Master Arm Passthrough Example"
    "\n================================================================================"
    "\n"
    "\nAn example application for passing data from a master arm to a slave manipulator "
    "\nvía serial. Ensure both devices are connected and powered on before starting."
    "\n"
    "\nUsage:"
    "\n  python3 ./examples/master_arm_passthrough.py -mp COM1 -sp COM2"
    "\n"
    "\nDISCLAIMER: this application is for demonstration purposes only and is not intended"
    "\nto be a complete solution.\n"
)

def get_serial_ports():
    parser = argparse.ArgumentParser(description="Set master and slave manipulator serial ports.")
    parser.add_argument('-mp', '--master_port', type=str, default="COM6",
                        help="Serial port for master arm (default: COM6)")
    parser.add_argument('-sp', '--slave_port', type=str, default="COM13",
                        help="Serial port for slave manipulator (default: COM13)")
    args = parser.parse_args()
    return args.master_port, args.slave_port

def main():
    print(STARTUP_MESSAGE)
    
    master_port, slave_port = get_serial_ports()
    try:
        master_arm_serial = serial.Serial(master_port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)
        slave_serial = serial.Serial(slave_port, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)
    except Exception as e:
        print(f"Error opening serial ports: {e}")
        sys.exit(1)

    print(f"Beginning passthrough from master arm ({master_port}) to slave manipulator ({slave_port})")
    while True:
        time.sleep(0.01)
        try:
            data = master_arm_serial.read(2048)
        except Exception as e:
            print(f"Error Reading from master arm serial port: {e}")
            data = b''

        if data != b'':
            try:
                slave_serial.write(data)
            except Exception as e:
                print(f"Error Writing to slave manipulator serial port: {e}")

if __name__ == '__main__':
    main()

