
from enum import IntEnum

class PacketID(IntEnum):
    """
    Class containing BPL packet IDs.
    Look in The Serial Protocol Document for comprehensive details on packet ids.

    Packet IDs:

    """
    MODE = 0x01
    "1 byte - Describes the Mode of a device"
    VELOCITY = 0x02
    "1 float - Describes the velocity of a device. Radians/s for angular joints. mm/s for linear joints."
    POSITION = 0x03
    "1 float - Describes the position of a device. In radians or mm"
    CURRENT = 0x05
    "1 float - Describes the current drawn by a device in mA"
    RELATIVE_POSITION = 0x0E
    "1 float - When sent sets the relative position of actuator. The actuator will move from its current position the amount specified in the data."
    INDEXED_POSITION = 0x0D
    "1 float - On first receiving indexed position an offset is created between the indexed position demand received and the current position. New indexed positions packets then move the actuators relative to the initial indexed position. "
    REQUEST = 0x60
    "bytes - Request a packet ID. On receiving the command, the device will send the packet corresponding to the packet IDs in the data field."
    SERIAL_NUMBER = 0x61
    "1 float - The unique serial number of the device"
    MODEL_NUMBER = 0x62
    "1 float - The model number of the device"
    TEMPERATURE = 0x66
    "1 float - The internal temperature in Celsius"
    SOFTWARE_VERSION = 0x6C
    "3 bytes - The software version on the device"
    KM_END_POS = 0xA1
    "6 floats - Request the current end effector position. (X, Y, Z, Y, P, R) in mm and radians. Only for kinematic enabled arms."
    KM_END_VEL = 0xA2
    "6 floats - Demand the end effector velocity (XYZ, RZ, RY, RX) in mm/s and rads/s. Only for kinematic enabled arm. Rotation commands (RZ, RY, RX) is only available for 7 function arms."
    KM_END_VEL_LOCAL = 0xCB
    "6 floats - Demand the end effector velocity relative to the end effector. (XYZ, RZ, RY, RX) in mm/s and rads/s. Only fora kinematic enabled arm. Rotation commands (RZ, RY, RX) is only available for 7 function arms."
    KM_BOX_OBSTACLE_02 = 0xA5
    "6 floats - (X1, Y1, Z1, X2, Y2, Z2) mm. Box obstacle defined by 2 opposite corners of a rectangular prism. "
    KM_BOX_OBSTACLE_03 = 0xA6
    KM_BOX_OBSTACLE_04 = 0xA7
    KM_BOX_OBSTACLE_05 = 0xA8
    KM_CYLINDER_OBSTACLE_02 = 0xAB
    "7 floats - (X1, Y1, Z1, X2, Y2, Z2, R) mm. Cylinder obstacle defined by 2 opposite centers of a cylinder. R defining the radius of the cylinder"
    KM_CYLINDER_OBSTACLE_03 = 0xAC
    KM_CYLINDER_OBSTACLE_04 = 0xAD
    KM_CYLINDER_OBSTACLE_05 = 0xAE

    VOLTAGE = 0x90
    "1 float - The supply voltage in Volts"
    SAVE = 0x50
    "1 byte - Send this to save user configurable settings on a device"
    HEARTBEAT_FREQUENCY = 0x92
    "1 byte - set the frequency of a packet to be sent from a device."
    HEARTBEAT_SET = 0x91
    "10 bytes - Specify the Packet IDs to be sent via heartbeat."
    POSITION_LIMITS = 0x10
    "2 floats - Maximum and Minimum positions of the device"
    VELOCITY_LIMITS = 0x11
    "2 floats - Maximum and Minimum velocities of the device"
    CURRENT_LIMITS = 0x12
    "2 floats - Maximum and Minimum currents of the device"

    ATI_FT_READING = 0xD8
    "6 floats - Read force in N and Torque in Nm from the Force torque sensor. (FX, FY, FZ, TX, TY, TZ). Send this packet to the FT Sensor to Tare it"
    BOOTLOADER = 0xFF

    VOLTAGE_THRESHOLD_PARAMETERS = 0x99
    "4 floats - Parameters to define the voltage threshold parameters. Enabled (0 or 1), V_Min (V), V_Max (V), time (seconds)."

