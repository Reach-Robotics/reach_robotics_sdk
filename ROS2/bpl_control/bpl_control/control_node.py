# Makind this for the bravo 7 for now.

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from bpl_msgs.msg import Packet

from bplprotocol import BPLProtocol, PacketID

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import math

from scipy.spatial.transform import Rotation as R

class Mode:
    STANDBY = 0
    POSITION = 1
    VELOCITY = 2
    KM_VEL = 3
    KM_VEL_LOCAL = 4
    KM_POS = 5

class BPLControlNode(Node):
    mode = Mode.STANDBY

    def __init__(self):
        super().__init__("bpl_control_node")

        self.declare_parameter('joints', [0x01, 0x02, 0x03, 0x04, 0x05])
        # self.declare_parameter('joint_names', ['bravo_axis_a', 'bravo_axis_b', 'bravo_axis_c', 'bravo_axis_d', 'bravo_axis_e', 'bravo_axis_f', 'bravo_axis_g'])
        self.declare_parameter('timeout', 0.2)  # timeout before, zero velocities are set to the joints.
        self.declare_parameter('publish_frequency', 20)

        self.joints = self.get_parameter('joints').value
        # self.joint_names = self.get_parameter('joint_names').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        
        self.tx_publisher = self.create_publisher(Packet, "tx", 10)

        self.joint_command_subscriber       = self.create_subscription(Float32MultiArray, "command/joint_positions", self.position_command_handler, 10)
        self.velocity_command_subscriber    = self.create_subscription(Float32MultiArray, "command/velocities", self.velocity_command_handler, 10)
        self.km_command_subscriber          = self.create_subscription(PoseStamped, "command/km_command", self.km_command_handler, 10)


        self.position_command = [float("NAN")] * len(self.joints)
        self.velocity_command = [float(0.0) * len(self.joints)] # Can't seen NAN with Float32MultiArray
        # self.position_command = [0.001, 3.2, 0.9, 3.0, 3.02]
        self.km_command = None
        # self.request_packet = Packet()
        # self.request_packet.device_id = 0xFF
        # self.request_packet.packet_id = int(PacketID.REQUEST)
        # self.request_packet.data = self.packet_ids

        # self.timer = self.create_timer(1/self.request_frequency, self.request_data)
        self.timer2 = self.create_timer(1/self.publish_frequency, self.publish_command)

    def position_command_handler(self, positions):
        self.position_command = positions.data

        self.position_command[0] = self.position_command[0] * 1000
        self.mode = Mode.POSITION
        pass
    def velocity_command_handler(self, velocities):

        velocity_limit_ms   = 0.01 # 1cm/s
        velocity_limit_rads = 0.15  # 11.5 deg/s

        
 
        velocities = list(velocities.data)
        self.mode = Mode.VELOCITY

        # Let's do some basic format checks and safety stuff
        if len(velocities) != len(self.joints):
            # Bad size, probably trying to use the 7F instead
            print(f"Size wrong! Expected: {len(self.joints)}, Got: {len(velocities)}. Dropping")
            return


        for index, velocity in enumerate(velocities):
            
            # Let's make sure we don't exceed a safety speed while testing
            # Assume EE at index 0
            if index == 0:
                velocity_limit = velocity_limit_ms
            else:
                velocity_limit = velocity_limit_rads

            if velocity > velocity_limit:
                velocities[index] = velocity_limit
            if velocity < -velocity_limit:
                velocities[index] = -velocity_limit

        if len(velocities) == len(self.joints):
            self.velocity_command = velocities


            # self.velocity_command = 

    def km_command_handler(self, km_pos: PoseStamped):
        # Ensure to convert the quaternion to the appropiate km command.

        # initall assume it has the correct 
        x = km_pos.pose.position.x * 1000
        y = km_pos.pose.position.y * 1000
        z = km_pos.pose.position.z * 1000

        quat = km_pos.pose.orientation.x, km_pos.pose.orientation.y, km_pos.pose.orientation.z, km_pos.pose.orientation.w
        rot = R.from_quat(quat).as_euler('xyz')
        self.km_command = [x, y, z, rot[2], rot[1], rot[0]]
        self.mode = Mode.KM_POS

    def publish_command(self):
        if self.mode == Mode.POSITION:
            for device_id, position in zip(self.joints, self.position_command):
                if math.isnan(position):
                    continue
                else:
                    p = Packet()
                    p.device_id = device_id
                    p.packet_id = PacketID.POSITION
                    p.data = list(BPLProtocol.encode_floats([position]))
                    # print(f"Pusblishing {device_id}, {position}")
                    self.tx_publisher.publish(p)

        elif self.mode == Mode.VELOCITY:
            for device_id, velocity in zip(self.joints, self.velocity_command):
                if device_id == 0x01:
                    velocity *= 1000
                p = Packet()
                p.device_id = device_id
                p.packet_id = PacketID.VELOCITY
                p.data = list(BPLProtocol.encode_floats([velocity]))

                print(f"Publishing {device_id}, {velocity}")
                self.tx_publisher.publish(p)

        elif self.mode == Mode.KM_POS and self.km_command is not None:
            p = Packet()
            p.packet_id = PacketID.KM_END_POS
            p.device_id = 0x0E
            p.data = list(BPLProtocol.encode_floats(self.km_command))
            # print(f"Publishing {p.data }")
            self.tx_publisher.publish(p)
        else:
            for device_id in self.joints:
                p = Packet()
                p.device_id = device_id
                p.packet_id = PacketID.VELOCITY
                p.data = list(BPLProtocol.encode_floats([0.0]))
                #self.tx_publisher.publish(p)
            self.position_command = [float("NAN")] * len(self.joints)
            self.km_command = None

            

def main(args=None):
    rclpy.init(args=args)
    bcn = BPLControlNode()
    rclpy.spin(bcn)


if __name__ == "__main__":
    main()