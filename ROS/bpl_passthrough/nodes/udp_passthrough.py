#!/usr/bin/env python
"""
serial_passthrough.py

Used to connect to an arm, and forward and ros messages received and
"""

import rospy
from bpl_msgs.msg import Packet
import socket

from bplprotocol import PacketReader, BPLProtocol


class BPLPassthroughNode:
    def __init__(self):

        self.rx_publisher = rospy.Publisher('rx', Packet, queue_size=100)
        rospy.init_node('udp_passthrough', anonymous=True)
        self.tx_subscriber = rospy.Subscriber("tx", Packet, self.tx_transmit)

        self.ip_address = rospy.get_param('~ip_address', default='192.168.2.3')
        self.port = rospy.get_param('~port', default=6789)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("", 0))
        rospy.loginfo("Opened Socket to ip {} at port {}".format(self.ip_address, self.port))
        self.rx_receive()
        pass

    def tx_transmit(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)
        rospy.logdebug("Transmitting {}, {}, {}".format(device_id, packet_id, data))

        encoded_packet = BPLProtocol.encode_packet(device_id, packet_id, data)
        self.sock.sendto(encoded_packet, (self.ip_address, self.port))

    def rx_receive(self):
        packet_reader = PacketReader()
        # rate = rospy.Rate(10000)
        while not rospy.is_shutdown():

            data = bytearray([])
            for i in range(100):
                try:
                    _data, address = self.sock.recvfrom(4096)

                    data = data + bytearray(_data)
                except socket.error as e:
                    break
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
                    # rospy.loginfo("Publishing {}".format(ros_packet))
                    self.rx_publisher.publish(ros_packet)

            # rate.sleep()
            pass


if __name__ == '__main__':
    passthrough_node = BPLPassthroughNode()
