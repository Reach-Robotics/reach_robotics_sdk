"""
serial_passthrough.py

Used to connect to an arm, and forward and ros messages received and
"""

import rclpy
from rclpy.node import Node

from bpl_msgs.msg import Packet

from bplprotocol import PacketReader, BPLProtocol
import serial
import time


class BPLPassthroughNode(Node):

    def __init__(self):
        super().__init__('serial_passthrough')
        self.rx_publisher = self.create_publisher(Packet,'rx', 5)

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('parity', serial.PARITY_NONE)
        self.declare_parameter('stop_bits', serial.STOPBITS_ONE)

        serial_port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        parity = self.get_parameter('parity').value
        stop_bits = self.get_parameter('stop_bits').value
        
        self.tx_subscriber = self.create_subscription(Packet, "tx", self.tx_transmit, 5)

        try:
            self.serial_port = serial.Serial(serial_port, baudrate=baudrate, parity=parity,
                                    stopbits=stop_bits, timeout=0)
            self.get_logger().info("Opened serial port {}".format(serial_port))
        except serial.SerialException as e:
            self.get_logger().error("Unable to open serial port {}".format(e))
            self.destroy_node()
            return
        self.packet_reader = PacketReader()
        self.timer = self.create_timer(1/10000000, self.rx_receive)


        self.rx_receive()
        pass

    def tx_transmit(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)
        self.get_logger().debug("Transmitting {}, {}, {}".format(device_id, packet_id, data))

        encoded_packet = BPLProtocol.encode_packet(device_id, packet_id, data)
        # print("Send")
        try:   
            
            self.serial_port.write(encoded_packet)
        except serial.SerialException as e:
            self.get_logger().warn("Unable to write to serial")

    def rx_receive(self):
        
        # rate = rospy.Rate(10000)
        # while rclpy.ok():
        #     rclpy.spin_once(self)
            # time.sleep(0.000000001)
        try:
            data = bytearray(self.serial_port.read())
        except serial.SerialException:
            self.get_logger().debug("Error reading from serial")
            return

        if data:
            packets = self.packet_reader.receive_bytes(data)

            for packet in packets:
                device_id = packet[0]
                packet_id = packet[1]
                data = packet[2]

                ros_packet = Packet()
                ros_packet.device_id = device_id
                ros_packet.packet_id = packet_id
                ros_packet.data = list(data)
                self.rx_publisher.publish(ros_packet)
        # Read from serial and transmit
        pass


def main(args=None):
    rclpy.init(args=args)
    passthrough_node = BPLPassthroughNode()
    rclpy.spin(passthrough_node)


if __name__ == "__main__":
    main()