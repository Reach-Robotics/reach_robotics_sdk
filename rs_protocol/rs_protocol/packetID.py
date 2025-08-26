class PacketID():
    ''' Please read the **Reach System - Communication Protocol** documentation for more information '''
    PacketType = dict()

    # Request
    REQUEST = 0x60
    PacketType[REQUEST] = int

    HEARTBEAT_PACKETS = 0x91
    PacketType[HEARTBEAT_PACKETS] = int

    HEARTBEAT_FREQUENCY_HZ = 0x92
    PacketType[HEARTBEAT_FREQUENCY_HZ] = int

    # Control
    MODE = 0x01
    PacketType[MODE] = int

    RESET = 0xFD
    PacketType[RESET] = int

    VELOCITY = 0x02
    PacketType[VELOCITY] = float

    POSITION = 0x03
    PacketType[POSITION] = float

    CURRENT = 0x05
    PacketType[CURRENT] = float

    TORQUE = 0x0B
    PacketType[TORQUE] = float

    INDEXED_RELATIVE_POSITION = 0x0D
    PacketType[INDEXED_RELATIVE_POSITION] = float

    INVERSE_KINEMATICS_GLOBAL_POSITION = 0xA1
    PacketType[INVERSE_KINEMATICS_GLOBAL_POSITION] = float

    INVERSE_KINEMATICS_GLOBAL_VELOCITY = 0xA2
    PacketType[INVERSE_KINEMATICS_GLOBAL_VELOCITY] = float

    INVERSE_KINEMATICS_LOCAL_VELOCITY = 0xCB
    PacketType[INVERSE_KINEMATICS_LOCAL_VELOCITY] = float

    REQUEST_JAW_CALIBRATION = 0x38
    PacketType[REQUEST_JAW_CALIBRATION] = float

    # Preset Positions
    POSITION_PRESET_GO = 0x55
    PacketType[POSITION_PRESET_GO] = int

    POSITION_PRESET_CAPTURE = 0x56
    PacketType[POSITION_PRESET_CAPTURE] = int

    POSITION_PRESET_SET_0 = 0x57
    PacketType[POSITION_PRESET_SET_0] = float

    POSITION_PRESET_SET_1 = 0x58
    PacketType[POSITION_PRESET_SET_1] = float

    POSITION_PRESET_SET_2 = 0x59
    PacketType[POSITION_PRESET_SET_2] = float

    POSITION_PRESET_SET_3 = 0x5A
    PacketType[POSITION_PRESET_SET_3] = float

    POSITION_PRESET_NAME_0 = 0x5B
    PacketType[POSITION_PRESET_NAME_0] = float

    POSITION_PRESET_NAME_1 = 0x5C
    PacketType[POSITION_PRESET_NAME_1] = float

    POSITION_PRESET_NAME_2 = 0x5D
    PacketType[POSITION_PRESET_NAME_2] = float

    POSITION_PRESET_NAME_3 = 0x5E
    PacketType[POSITION_PRESET_NAME_3] = float

    # Configure
    SAVE = 0x50
    PacketType[SAVE] = int

    POSITION_LIMITS = 0x10
    PacketType[POSITION_LIMITS] = float

    VELOCITY_LIMITS = 0x11
    PacketType[VELOCITY_LIMITS] = float

    CURRENT_LIMITS = 0x12
    PacketType[CURRENT_LIMITS] = float

    COMPUTE_IP_ADDRESS = 0x33 # TODO: add to protocol documentation
    PacketType[COMPUTE_IP_ADDRESS] = int

    ROUTER_ETH_PARAMETERS = 0x46 # TODO: add to prototol documentation
    PacketType[ROUTER_ETH_PARAMETERS] = int

    MOUNT_TRANSFORM = 0xB4  # TODO: add to protocol documentation
    PacketType[MOUNT_TRANSFORM] = float

    DEVICE_TYPE = 0x67  # TODO: add to protocol documentation
    PacketType[DEVICE_TYPE] = int

    # Workspace Restrictions
    BOX_OBSTACLE_02 = 0xA5
    PacketType[BOX_OBSTACLE_02] = float

    BOX_OBSTACLE_03 = 0xA6
    PacketType[BOX_OBSTACLE_03] = float

    BOX_OBSTACLE_04 = 0xA7
    PacketType[BOX_OBSTACLE_04] = float

    BOX_OBSTACLE_05 = 0xA8
    PacketType[BOX_OBSTACLE_05] = float

    CYLINDER_OBSTACLE_02 = 0xAB
    PacketType[CYLINDER_OBSTACLE_02] = float

    CYLINDER_OBSTACLE_03 = 0xAC
    PacketType[CYLINDER_OBSTACLE_03] = float

    CYLINDER_OBSTACLE_04 = 0xAD
    PacketType[CYLINDER_OBSTACLE_04] = float

    CYLINDER_OBSTACLE_05 = 0xAE
    PacketType[CYLINDER_OBSTACLE_05] = float

    # Monitor
    VOLTAGE = 0x90
    PacketType[VOLTAGE] = float

    INTERNAL_HUMIDITY = 0x65
    PacketType[INTERNAL_HUMIDITY] = float

    INTERNAL_TEMPERATURE = 0x66
    PacketType[INTERNAL_TEMPERATURE] = float

    INTERNAL_PRESSURE = 0x6E
    PacketType[INTERNAL_PRESSURE] = float

    FACTORY_CLIMATE = 0x28
    PacketType[FACTORY_CLIMATE] = float

    SOFTWARE_VERSION = 0x6C
    PacketType[SOFTWARE_VERSION] = int

    FORCE_TORQUE_SENSOR_READING = 0xD8
    PacketType[FORCE_TORQUE_SENSOR_READING] = float

    DEVICE_ID = 0x64  # TODO: add to protocol documentation
    PacketType[DEVICE_ID] = int

    HARDWARE_STATUS_FLAG = 0x68
    PacketType[HARDWARE_STATUS_FLAG] = int





