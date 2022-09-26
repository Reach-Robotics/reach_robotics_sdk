#!/usr/bin/env python
"""
serial_passthrough.py

Used to connect to an arm, and forward and ros messages received and
"""

import rospy
from bpl_msgs.msg import Packet
import serial

from bplprotocol import PacketReader, BPLProtocol



class BPLPassthroughNode:

    def __init__(self):

        self.rx_publisher = rospy.Publisher('rx', Packet, queue_size=100)
        rospy.init_node('serial_passthrough', anonymous=True)

        serial_port = rospy.get_param('~serial_port', default='/dev/ttyUSB1')
        baudrate = rospy.get_param('~baudrate', default=115200)
        parity = rospy.get_param('~parity', default=serial.PARITY_NONE)
        stop_bits = rospy.get_param('~stop_bits', default=serial.STOPBITS_ONE)

        self.tx_subscriber = rospy.Subscriber("tx", Packet, self.tx_transmit)

        try:
            self.serial_port = serial.Serial(serial_port, baudrate=baudrate, parity=parity,
                                    stopbits=stop_bits, timeout=0)
            rospy.loginfo("Opened serial port {}".format(serial_port))
        except serial.SerialException as e:
            rospy.logerr("Unable to open serial port {}".format(e))
            return

        self.rx_receive()
        pass

    def tx_transmit(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)
        rospy.logdebug("Transmitting {}, {}, {}".format(device_id, packet_id, data))

        encoded_packet = BPLProtocol.encode_packet(device_id, packet_id, data)
        try:
            self.serial_port.write(encoded_packet)
        except serial.SerialException as e:
            rospy.logwarn("Unable to write to serial")

    def rx_receive(self):
        packet_reader = PacketReader()
        # rate = rospy.Rate(10000)
        while not rospy.is_shutdown():

            try:
                data = bytearray(self.serial_port.read())
            except serial.SerialException:
                rospy.logdebug("Error reading from serial")
                continue

            if data:
                packets = packet_reader.receive_bytes(data)

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


if __name__ == '__main__':
    passthrough_node = BPLPassthroughNode()