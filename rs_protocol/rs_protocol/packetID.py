class PacketID():
    ''' Please read the **Reach System - Communication Protocol** documentation for more information '''
    PacketType = dict()

    # Request
    REQUEST = 0x60
    HEARTBEAT_SET = 0x91
    HEARTBEAT_FREQUENCY = 0x92

    PacketType[REQUEST] = int
    PacketType[HEARTBEAT_SET] = int
    PacketType[HEARTBEAT_FREQUENCY] = int
    
    # Control 
    MODE = 0x01

    VELOCITY = 0x02 
    POSITION = 0x03
    CURRENT = 0x05 

    INDEXED_RELATIVE_POSITION = 0x0D

    INVERSE_KINEMATICS_GLOBAL_POSITION = 0xA1
    INVERSE_KINEMATICS_GLOBAL_VELOCITY = 0xA2 
    INVERSE_KINEMATICS_LOCAL_VELOCITY = 0xCB

    PacketType[MODE] = int
    PacketType[VELOCITY] = float
    PacketType[POSITION] = float
    PacketType[CURRENT] = float
    PacketType[INDEXED_RELATIVE_POSITION] = float
    PacketType[INVERSE_KINEMATICS_GLOBAL_POSITION] = float
    PacketType[INVERSE_KINEMATICS_GLOBAL_VELOCITY] = float
    PacketType[INVERSE_KINEMATICS_LOCAL_VELOCITY] = float
    
    # Preset Positions
    POSITION_PRESET_GO = 0x55
    POSITION_PRESET_CAPTURE = 0x56

    POSITION_PRESET_SET_0 = 0x57
    POSITION_PRESET_SET_1 = 0x58
    POSITION_PRESET_SET_2 = 0x59
    POSITION_PRESET_SET_3 = 0x5A

    POSITION_PRESET_NAME_0 = 0x5B
    POSITION_PRESET_NAME_1 = 0x5C
    POSITION_PRESET_NAME_2 = 0x5D
    POSITION_PRESET_NAME_3 = 0x5E

    PacketType[POSITION_PRESET_GO] = int
    PacketType[POSITION_PRESET_CAPTURE] = int
    PacketType[POSITION_PRESET_SET_0] = float
    PacketType[POSITION_PRESET_SET_1] = float
    PacketType[POSITION_PRESET_SET_2] = float
    PacketType[POSITION_PRESET_SET_3] = float
    PacketType[POSITION_PRESET_NAME_0] = float
    PacketType[POSITION_PRESET_NAME_1] = float
    PacketType[POSITION_PRESET_NAME_2] = float
    PacketType[POSITION_PRESET_NAME_3] = float

    # Configure
    SAVE = 0x50 
    POSITION_LIMITS = 0x10 
    VELOCITY_LIMITS = 0x11
    CURRENT_LIMITS = 0x12

    PacketType[SAVE] = int
    PacketType[POSITION_LIMITS] = float
    PacketType[VELOCITY_LIMITS] = float
    PacketType[CURRENT_LIMITS] = float

    # Workspace Restrictions
    BOX_OBSTACLE_02 = 0xA5 
    BOX_OBSTACLE_03 = 0xA6
    BOX_OBSTACLE_04 = 0xA7
    BOX_OBSTACLE_05 = 0xA8

    CYLINDER_OBSTACLE_02 = 0xAB
    CYLINDER_OBSTACLE_03 = 0xAC
    CYLINDER_OBSTACLE_04 = 0xAD
    CYLINDER_OBSTACLE_05 = 0xAE

    PacketType[BOX_OBSTACLE_02] = float
    PacketType[BOX_OBSTACLE_03] = float
    PacketType[BOX_OBSTACLE_04] = float
    PacketType[BOX_OBSTACLE_05] = float
    PacketType[CYLINDER_OBSTACLE_02] = float
    PacketType[CYLINDER_OBSTACLE_03] = float
    PacketType[CYLINDER_OBSTACLE_04] = float
    PacketType[CYLINDER_OBSTACLE_05] = float

    # Monitor
    VOLTAGE = 0x90 
    INTERNAL_HUMIDITY = 0x65
    INTERNAL_TEMPERATURE = 0x66 
    INTERNAL_PRESSURE = 0x67
    FACTORY_CLIMATE = 0x28
    SOFTWARE_VERSION = 0x6C 
    HARDWARE_STATUS_FLAG = 0x68
    
    PacketType[VOLTAGE] = float
    PacketType[INTERNAL_HUMIDITY] = float
    PacketType[INTERNAL_TEMPERATURE] = float
    PacketType[INTERNAL_PRESSURE] = float
    PacketType[FACTORY_CLIMATE] = float
    PacketType[SOFTWARE_VERSION] = int
    PacketType[HARDWARE_STATUS_FLAG] = int
        

    
    
    
