from enum import IntEnum

class Mode(IntEnum):
    ''' Please read the **Reach System - Communication Protocol** Documentation for more information '''
    STANDBY = 0x00
    DISABLED = 0x01 
    POSITION = 0x02 
    VELOCITY = 0x03 
    CURRENT = 0x04
    TORQUE = 0x30
    INDEXED_RELATIVE_POSITION = 0x13
    POSITION_PRESET = 0x14
    ZERO_VELOCITY = 0x15
    KINEMATIC_POSITION_BASE_FRAME = 0x17
    KINEMATIC_VELOCTIY_BASE_FRAME = 0x18
    KINEMATIC_VELOCTIY_END_EFFECTOR_FRAME = 0x1A
    POSITION_VELOCITY = 0x1C
    POSITION_HOLD = 0x1D
    PASSIVE = 0x26 