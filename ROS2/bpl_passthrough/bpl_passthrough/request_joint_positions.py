"""ROS/bpl_passthrough/scripts/request_joint_positions.py"""

import rclpy
from rclpy.node import Node
from bpl_msgs.msg import Packet

from bplprotocol import BPLProtocol, PacketID
import time

class JointRequestExample(Node):

    def __init__(self):
        super().__init__("request_joint_position_script")

        self.declare_parameter("~frequency", 20)

        self.frequency = 20

        self.tx_publisher = self.create_publisher(Packet, "tx", 100)
        self.rx_subscriber = self.create_subscription(Packet, "rx", self.receive_packet, 100)

        self.request_packet = Packet()
        self.request_packet.device_id = 0xFF
        self.request_packet.packet_id = int(PacketID.REQUEST)
        self.request_packet.data = [PacketID.POSITION]

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
                
    def timer_callback(self):
        self.tx_publisher.publish(self.request_packet)

    def receive_packet(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)

        if packet_id == PacketID.POSITION:
            position = BPLProtocol.decode_floats(data)[0]
            self.get_logger().info("Position Received: {} - {}".format(device_id, position))

def main(args=None):
    rclpy.init(args=args)
    jre = JointRequestExample()
    rclpy.spin(jre)


if __name__ == "__main__":
    main()