#!/usr/bin/env python

"""ROS/bpl_passthrough/scripts/request_joint_positions.py"""

import rospy
from bpl_msgs.msg import Packet

from bplprotocol import BPLProtocol, PacketID


def receive_packet(packet):
    device_id = packet.device_id
    packet_id = packet.packet_id
    data = bytearray(packet.data)

    if packet_id == PacketID.POSITION:
        position = BPLProtocol.decode_floats(data)[0]
        print("Position Received: {} - {}".format(device_id, position))


if __name__ == '__main__':
    tx_publisher = rospy.Publisher("tx", Packet, queue_size=100)
    rospy.init_node("request_joint_position_script")
    frequency = rospy.get_param('~frequency', default=20)
    rx_subscriber = rospy.Subscriber("rx", Packet, receive_packet)

    request_packet = Packet(0xFF, PacketID.REQUEST, [PacketID.POSITION])

    rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        tx_publisher.publish(request_packet)
        rate.sleep()
