import socket
import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import RSProtocol, create_socket_connection, create_serial_connection

LOOP_FREQUENCY = 500  # Hz
QUEUE_SIZE = 100

class RSPassthroughNode(Node):

    def __init__(self):
        super().__init__('udp_passthrough')

        self.rx_publisher = self.create_publisher(Packet,'rx', QUEUE_SIZE)
        self.tx_subscriber = self.create_subscription(Packet, 'tx', self.tx_transmit, QUEUE_SIZE)

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('half_duplex', False)

        serial_port = self.get_parameter('serial_port').value
        half_duplex = self.get_parameter('half_duplex').value

        self.rs_protocol = RSProtocol(create_serial_connection(serial_port), half_duplex=half_duplex)
        self.get_logger().info(f"Opened Serial Port {serial_port} with half duplex: {half_duplex}")

        self.timer = self.create_timer(1/LOOP_FREQUENCY, self.rx_receive)

    def tx_transmit(self, ros_packet):
        data = list(ros_packet.int_data) if ros_packet.int_data else list(ros_packet.float_data)
        self.rs_protocol.write(ros_packet.device_id, ros_packet.packet_id, data)
        self.get_logger().debug(f"Transmit: {ros_packet}")

    def rx_receive(self):
        packets = self.rs_protocol.read()

        for packet in packets:
            ros_packet = Packet()
            ros_packet.device_id = packet[0]
            ros_packet.packet_id = packet[1]
            if isinstance(packet[2][0], int):
                ros_packet.int_data = packet[2]
            else:
                ros_packet.float_data = packet[2]

            self.get_logger().debug(f"Receive: {ros_packet}")
            self.rx_publisher.publish(ros_packet)

def main(args=None):
    rclpy.init(args=args)
    passthrough_node = RSPassthroughNode()
    rclpy.spin(passthrough_node)

if __name__ == '__main__':
    main()
