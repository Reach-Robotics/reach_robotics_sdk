import rclpy
from rclpy.node import Node
from rs_msgs.msg import Packet
from rs_protocol import RSProtocol, PacketID

LOOP_FREQUENCY = 20  # Hz
QUEUE_SIZE = 100

class State:
    q: float = 0.0    # Position [rad]/[mm]
    dq: float = 0.0   # Velocity [rad/s]/[mm/s]
    Iq: float = 0.0   # Quadrature Current [mA]
    tau: float = 0.0  # Torque [Nm]/[N]  (Note: torque is not available on Reach Alpha devices)

class Actuator():
    def __init__(self, node: Node, device_id: int) -> None:
        self.device_id = device_id
        self.state = State()
        self.node = node

        self.node.create_subscription(Packet, "rx", self.position_callback, QUEUE_SIZE)
        self.node.create_subscription(Packet, "rx", self.velocity_callback, QUEUE_SIZE)
        self.node.create_subscription(Packet, "rx", self.current_callback, QUEUE_SIZE)
        self.node.create_subscription(Packet, "rx", self.torque_callback, QUEUE_SIZE)

    def position_callback(self, packet):
        data = list(packet.int_data) if packet.int_data else list(packet.float_data)
        if packet.device_id == self.device_id and packet.packet_id == PacketID.POSITION:
            self.state.q = data[0]

    def velocity_callback(self, packet):
        data = list(packet.int_data) if packet.int_data else list(packet.float_data)
        if packet.device_id == self.device_id and packet.packet_id == PacketID.VELOCITY:
            self.state.dq = data[0]

    def current_callback(self, packet):
        data = list(packet.int_data) if packet.int_data else list(packet.float_data)
        if packet.device_id == self.device_id and packet.packet_id == PacketID.CURRENT:
            self.state.Iq = data[0]

    def torque_callback(self, packet):
        data = list(packet.int_data) if packet.int_data else list(packet.float_data)
        if packet.device_id == self.device_id and packet.packet_id == PacketID.TORQUE:
            self.state.tau = data[0]


class JointTelemetry(Node):

    def __init__(self):
        super().__init__("joint_telemetry")

        self.declare_parameter("~frequency", LOOP_FREQUENCY)
        self.frequency = self.get_parameter("~frequency").value
        self.tx_publisher = self.create_publisher(Packet, "tx", QUEUE_SIZE)

        self.actuators = []
        for device_id in range(1, 8):
            self.actuators.append(Actuator(self, device_id))

        self.telemetry_packet = Packet()
        self.telemetry_packet.device_id = 0xFF
        self.telemetry_packet.packet_id = PacketID.REQUEST
        self.telemetry_packet.int_data = [PacketID.POSITION, PacketID.VELOCITY, PacketID.CURRENT, PacketID.TORQUE]

        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
                
    def timer_callback(self):
        self.tx_publisher.publish(self.telemetry_packet)
        self.get_logger().info(f"q: {[round(actuator.state.q, 1) for actuator in self.actuators]}, |q̇|: {[abs(round(actuator.state.dq, 1)) for actuator in self.actuators]}")

def main(args=None):
    rclpy.init(args=args)
    jre = JointTelemetry()
    rclpy.spin(jre)


if __name__ == "__main__":
    main()