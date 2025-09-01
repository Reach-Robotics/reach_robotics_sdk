import logging
import numpy as np

from typing import List
from scipy.spatial.transform import Rotation as R

from rs_protocol import RSProtocol, PacketID
from device.device import Actuator, Router, Compute

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)


class Manipulator:
    def __init__(self, protocol: RSProtocol, num_actuators: int=7, router_id: int=0x0D, compute_id: int=0x0E) -> None:
        self.protocol = protocol
        self.actuators: List[Actuator] = []
        self.router: Router = Router(protocol, router_id)
        self.compute: Compute = Compute(protocol, compute_id)

        self.T_eg: np.ndarray = np.eye(4)
        self.T_bg: np.ndarray = np.eye(4)

        for i in range(1, num_actuators+1):
            self.actuators.append(Actuator(protocol, i))

        self.mount_transform_callback(self.router.device_id, 
                                      PacketID.MOUNT_TRANSFORM,
                                      self.protocol.request(self.router.device_id, PacketID.MOUNT_TRANSFORM))
        
        self.protocol.register_callback(self.router.device_id, 
                                        PacketID.MOUNT_TRANSFORM, 
                                        self.mount_transform_callback)
        self.protocol.register_callback(self.compute.device_id, 
                                        PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION, 
                                        self.eg_transform_callback)
    
    def update(self) -> None:       
        self.protocol.write(0xFF, PacketID.REQUEST, [PacketID.POSITION, 
                                                     PacketID.VELOCITY, 
                                                     PacketID.CURRENT, 
                                                     PacketID.TORQUE, 
                                                     PacketID.MODE,
                                                     PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION])
        self.protocol.read()

    def q(self) -> np.ndarray:
        return np.array([actuator.state.q for actuator in self.actuators])
    
    def dq(self) -> np.ndarray:
        return np.array([actuator.state.dq for actuator in self.actuators])
    
    def tau(self) -> np.ndarray:
        return np.array([actuator.state.tau for actuator in self.actuators])
    
    def i(self) -> np.ndarray:
        return np.array([actuator.state.Iq for actuator in self.actuators])
    
    def mode(self) -> np.ndarray:
        return np.array([actuator.mode for actuator in self.actuators]+[self.router.mode, self.compute.mode])
    
    def get_end_effector_transform(self) -> np.ndarray:
        """ 
        Returns the cached end-effector to global frame transform. 
        NOTE: this value is not updated unless a call to update() is made.
        """
        return self.T_eg
    
    def get_base_transform(self) -> np.ndarray:
        """ 
        Returns the cached base to global frame transform. 
        NOTE: this value is not updated unless a call to update() is made.
        """
        return self.T_bg
    
    def eg_transform_callback(self, device_id, packet_id, data) -> None:
        if len(data) != 6:
            logger.error(f"Unexpected data length {len(data)} for packet ID {PacketID.INVERSE_KINEMATICS_GLOBAL_POSITION}. "
                         f"Discarding data.")
            return

        T_eg: np.ndarray = np.eye(4)
        T_eg[0:3, 3] = np.array(data[0:3]) / 1000
        T_eg[0:3, 0:3] = R.from_euler('ZYX', [data[3], data[4], data[5]]).as_matrix() # 'ZYX' for extrinsic rotations for YPR axes

        self.T_eg = T_eg

    def mount_transform_callback(self, device_id, packet_id, data) -> None:
        if len(data) != 6:
            logger.error(f"Unexpected data length {len(data)} for packet ID {packet_id}. "
                         f"Discarding data.")
            return
        
        T_bg = np.eye(4)
        T_bg[0:3, 3] = np.array(data[0:3]) / 1000
        T_bg[0:3, 0:3] = R.from_euler('zyx', data[3:6]).as_matrix()
        self.T_bg = T_bg
    
    def ee_velocity_global(self, V_eg) -> None:
        """ Send end-effector velocity command in global frame [X, Y, Z, RX, RY, RZ] (mm/s, rad/s)"""
        self.protocol.write(self.compute.device_id, 
                            PacketID.INVERSE_KINEMATICS_GLOBAL_VELOCITY, 
                            V_eg[0:3]+V_eg[3:6][::-1])
        
    def ee_velocity_local(self, V_eg) -> None:
        """ Send end-effector velocity command in "local" frame [X, Y, Z, RX, RY, RZ] (mm/s, rad/s)"""
        self.protocol.write(self.compute.device_id, 
                            PacketID.INVERSE_KINEMATICS_LOCAL_VELOCITY, 
                            V_eg[0:3]+V_eg[3:6][::-1])