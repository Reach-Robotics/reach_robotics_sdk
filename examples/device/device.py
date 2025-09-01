import logging
import numpy as np

from scipy.spatial.transform import Rotation as R

from rs_protocol import RSProtocol, PacketID, Mode

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)


class State:
    q: float = 0.0  # Position [rad]/[mm]
    dq: float = 0.0  # Velocity [rad/s]/[mm/s]
    Iq: float = 0.0  # Quadrature Current [mA]
    tau: float = 0.0  # Torque [Nm]/[N]  (Note: torque is not available on Reach Alpha devices)


class Device:    
    def __init__(self, protocol: RSProtocol, device_id: int) -> None:
        self.mode: int = Mode.NULL
        self.device_id: int = device_id
        self.hardware_status = [0]*4  # Hardware status values (a,b,c,d) - refer to documentation for more details
        self.protocol: RSProtocol = protocol
        
        self.protocol.register_callback(self.device_id, PacketID.MODE, self.mode_callback)

    def mode_callback(self, device_id, packet_id, data):
        self.mode = data[0]

    def hardware_status_callback(self, device_id, packet_id, data):
        if len(data) != 4:
            logger.error(f"Unexpected data length {len(data)} for packet ID {packet_id}. Discarding data.")
            return
        
        self.hardware_status = data

class Actuator(Device):
    def __init__(self, protocol: RSProtocol, device_id: int) -> None:
        super().__init__(protocol, device_id)
        
        self.state = State()

        self.protocol.register_callback(self.device_id, PacketID.POSITION, self.position_callback)
        self.protocol.register_callback(self.device_id, PacketID.VELOCITY, self.velocity_callback)
        self.protocol.register_callback(self.device_id, PacketID.CURRENT, self.current_callback)
        self.protocol.register_callback(self.device_id, PacketID.TORQUE, self.torque_callback)

    def position_callback(self, device_id, packet_id, data):
        self.state.q = data[0]

    def velocity_callback(self, device_id, packet_id, data):
        self.state.dq = data[0]

    def current_callback(self, device_id, packet_id, data):
        self.state.Iq = data[0]

    def torque_callback(self, device_id, packet_id, data):
        self.state.tau = data[0]


class Router(Device):
    """
    In the Reach Robotics system, the Router (or base) is a device that 
    routes power and communications through the manipulator. For the Reach 
    Bravo and Reach X lines, this device is at the base of the manipulator 
    (0x0D). For the Reach Alpha, this device is the last device in the chain 
    (0x05).

    The router device is also used for storing device configurations settings 
    that are applicable to the entire manipulator, such as the mount transform,
    and the ethernet configuration.
    """
    def __init__(self, protocol: RSProtocol, device_id: int=0x0D) -> None:       
        super().__init__(protocol, device_id)

        self.T_bg: np.ndarray = np.eye(4) # Homogeneous transform from base to global frame
        
        self.router_mac = [0]*6
        self.compute_ip = [0]*4
        self.udp_port = 0
        
        self.subnet = [0]*4
        self.gateway = [0]*4
        self.dhcp_mode = 0

        self.protocol.register_callback(self.device_id, 
                                        PacketID.ROUTER_ETH_PARAMETERS, 
                                        self.router_ethernet_parameters_callback)
        self.protocol.register_callback(self.device_id, 
                                        PacketID.COMPUTE_IP_ADDRESS, 
                                        self.compute_ip_address_callback)

    def router_ethernet_parameters_callback(self, device_id, packet_id, data):
        if len(data) != 23:
            logger.error(f"Unexpected data length {len(data)} for packet ID {packet_id}. "
                         f"Discarding data.")
            return
        
        self.mac_address = data[0:6]
        self.ip_address = data[6:10]
        self.subnet = data[10:14]
        self.gateway = data[14:18]
        self.dhcp_mode = data[22]

    def compute_ip_address_callback(self, device_id, packet_id, data):
        if len(data) != 4:
            logger.error(f"Unexpected data length {len(data)} for packet ID {packet_id}. "
                         f"Discarding data.")
            return
        self.compute_ip = data


class Compute(Device):
    """ 
    In the Reach Robotics system, the Compute unit is controller
    that enables the higher level functions of the robot. For the 
    Reach Bravo and Reach X lines, this device is a Linux based 
    computer at the base of the manipulator (0x0E). For the Reach 
    Alpha, this device is the last device in the chain (0x05).

    The compute device is typically only available on devices with 5
    or more actuators (i.e. 5 functions or more).
    """
    def __init__(self, protocol: RSProtocol, device_id: int=0x0E) -> None:
        super().__init__(protocol, device_id)