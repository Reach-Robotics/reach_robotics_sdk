"""
The purpose of this script is to directly passthrough commands from the master arm to an arm.

"""

# install pyserial with pip install pyserial

import time

import serial


MASTER_ARM_PORT = "COM1"

ARM_PORT = "COM2"

if __name__ == '__main__':

    master_arm_serial = serial.Serial(MASTER_ARM_PORT, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)

    arm_serial = serial.Serial(ARM_PORT, baudrate=115200, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=0)

    print("Beginning passthrough from master arm to arm")
    while True:
        time.sleep(0.0001)
        try:
            data = master_arm_serial.read()
        except BaseException as e:
            print(f"Error Reading from master arm serial port: {e}")
            data = b''

        if data != b'':
            try:
                arm_serial.write(data)
            except BaseException as e:
                print(f"Error Writing to an arms serial port: {e}")

