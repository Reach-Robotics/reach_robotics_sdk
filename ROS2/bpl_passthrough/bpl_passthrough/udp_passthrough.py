
"""
udp_passthrough.py

Used to connect to an arm, and forward and ros messages received and
"""

import rclpy
from rclpy.node import Node

from bpl_msgs.msg import Packet
import socket

from bplprotocol import PacketReader, BPLProtocol


class BPLPassthroughNode(Node):

    def __init__(self):
        super().__init__('udp_passthrough')

        self.rx_publisher = self.create_publisher(Packet,'rx', 5)

        self.tx_subscriber = self.create_subscription(Packet, 'tx', self.tx_transmit, 5)

        self.declare_parameter('ip_address', '192.168.2.3')
        self.declare_parameter('port', 6789)

        self.ip_address = self.get_parameter('ip_address').value
        self.port = self.get_parameter('port').value

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("", 0))

        self.get_logger().info("Opened Socket to ip {} at port {}".format(self.ip_address, self.port))
      
        self.timer = self.create_timer(1/10000000, self.rx_receive)
        
        self.rx_receive()
        pass

    def tx_transmit(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)
        # self.get_logger().debug("Transmitting {}, {}, {}".format(device_id, packet_id, data))

        encoded_packet = BPLProtocol.encode_packet(device_id, packet_id, data)
        self.sock.sendto(encoded_packet, (self.ip_address, self.port))

    def rx_receive(self):
        packet_reader = PacketReader()
        # rate = rospy.Rate(10000)
        data = b''
        try:
            _data, address = self.sock.recvfrom(4096)
            data = bytearray(_data)
        except socket.error as e:
            pass

        if data:
            # data = bytearray(data)
            packets = packet_reader.receive_bytes(data)

            for packet in packets:
                device_id = packet[0]
                packet_id = packet[1]
                data = packet[2]

                ros_packet = Packet()
                ros_packet.device_id = device_id
                ros_packet.packet_id = packet_id
                ros_packet.data = list(data)
                # print(data)
                # self.get_logger().info("Publishing {}".format(ros_packet))
                self.rx_publisher.publish(ros_packet)

        # rate.sleep()
        pass

def main(args=None):
    rclpy.init(args=args)
    passthrough_node = BPLPassthroughNode()
    rclpy.spin(passthrough_node)

if __name__ == '__main__':
    main()
