import rclpy
from rclpy.node import Node
from bpl_msgs.msg import Packet

from bplprotocol import BPLProtocol, PacketID
import time

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from scipy.spatial.transform import Rotation

class EndEffectorPosePublisher(Node):

    def __init__(self):
        super().__init__("EndEffectorPosePublisher")

        self.declare_parameter("frequency", 20)
        self.declare_parameter("frame_id", "base_link")
        self.frame_id = self.get_parameter("frame_id").value
        self.frequency = self.get_parameter("frequency").value

        self.tx_publisher = self.create_publisher(Packet, "tx", 100)
        self.rx_subscriber = self.create_subscription(Packet, "rx", self.receive_packet, 100)
        self.pose_publisher = self.create_publisher(PoseStamped, "end_effector_pose", 10)
        self.request_packet = Packet()
        self.request_packet.device_id = 0xFF
        self.request_packet.packet_id = int(PacketID.REQUEST)
        self.request_packet.data = [PacketID.KM_END_POS]

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)

    def timer_callback(self):
        self.tx_publisher.publish(self.request_packet)

    def receive_packet(self, packet):
        device_id = packet.device_id
        packet_id = packet.packet_id
        data = bytearray(packet.data)

        if packet_id == PacketID.KM_END_POS:
            end_pos = BPLProtocol.decode_floats(data)
            # self.get_logger().info(f"KM_END_POS Received: {end_pos}")
        
            self.convert_km_end_pos_to_pose(end_pos)
    
    def convert_km_end_pos_to_pose(self, end_pos):
        output_pose = PoseStamped()

        # Pose Position
        output_pose.pose.position.x = float(end_pos[0]*0.001)
        output_pose.pose.position.y = float(end_pos[1]*0.001)
        output_pose.pose.position.z = float(end_pos[2]*0.001)

        # rotation object
        rot = Rotation.from_euler('xyz', end_pos[-3:][::-1])
        # convert to quaternion (x, y, z, w)
        rot_quat = rot.as_quat()

        # Pose Orientation (Quaternion)
        output_pose.pose.orientation.x = rot_quat[0]
        output_pose.pose.orientation.y = rot_quat[1]
        output_pose.pose.orientation.z = rot_quat[2]
        output_pose.pose.orientation.w = rot_quat[3]

        output_pose.header = self.make_header()

        self.publish_pose(output_pose)

    def make_header(self):
        hdr = Header()
        hdr.frame_id = self.frame_id
        hdr.stamp = self.get_clock().now().to_msg()
        return hdr
    
    def publish_pose(self, pose):
        self.pose_publisher.publish(pose)
        # self.get_logger().info(f'Published pose: {pose}')
       

def main(args = None):
    rclpy.init(args=args)
    kepr = EndEffectorPosePublisher()
    rclpy.spin(kepr)

if __name__ == "__main__":
    main()