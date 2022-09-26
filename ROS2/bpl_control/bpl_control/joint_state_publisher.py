import rclpy
from rclpy.node import Node

from bpl_msgs.msg import Packet

from bplprotocol import BPLProtocol, PacketID

from sensor_msgs.msg import JointState

class BPLJointStatesPublisher(Node):

    def __init__(self):
        super().__init__("bpl_joint_state_publisher_node")

        self.declare_parameter('joints', [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07])
        self.declare_parameter('joint_names', ['bravo_axis_a', 'bravo_axis_b', 'bravo_axis_c', 'bravo_axis_d', 'bravo_axis_e', 'bravo_axis_f', 'bravo_axis_g'])
        self.declare_parameter('request_frequency', 10)
        self.declare_parameter('publish_frequency', 10)

        self.joints = self.get_parameter('joints').value
        self.joint_names = self.get_parameter('joint_names').value
        self.request_frequency = self.get_parameter('request_frequency').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.packet_ids = [PacketID.POSITION, PacketID.VELOCITY]
        
        self.joint_positions = [0.0] * len(self.joints)
        self.joint_velocities = [0.0] * len(self.joints)

        self.tx_publisher = self.create_publisher(Packet, "tx", 10)
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)

        self.rx_subscriber = self.create_subscription(Packet, "rx", self.request_handler, 10)

        self.request_packet = Packet()
        self.request_packet.device_id = 0xFF
        self.request_packet.packet_id = int(PacketID.REQUEST)
        self.request_packet.data = self.packet_ids

        self.timer = self.create_timer(1/self.request_frequency, self.request_data)
        self.timer2 = self.create_timer(1/self.publish_frequency, self.publish_joint_state)


    def request_data(self):
        self.tx_publisher.publish(self.request_packet)

    def publish_joint_state(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.joint_positions.copy()
        joint_state.velocity = self.joint_velocities.copy()
        self.joint_state_publisher.publish(joint_state)
        # print("send")

    def request_handler(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)

        if packet_id == PacketID.POSITION:
            if device_id in self.joints:

                device_index = self.joints.index(device_id)

                position = BPLProtocol.decode_floats(data)[0]
                
                if device_index == 0:
                    position = position*0.001
                self.joint_positions[device_index] = position
                
        if packet_id == PacketID.VELOCITY:
            if device_id in self.joints:

                device_index = self.joints.index(device_id)
                
                velocity = BPLProtocol.decode_floats(data)[0]
                if device_index == 0:
                    velocity = velocity*0.001
                self.joint_velocities[device_index] = velocity

def main(args=None):
    rclpy.init(args=args)
    jsp = BPLJointStatesPublisher()
    rclpy.spin(jsp)


if __name__ == "__main__":
    main()