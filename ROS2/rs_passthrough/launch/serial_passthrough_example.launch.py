from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution , LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    serial_port_launch_arg = DeclareLaunchArgument("serial_port", default_value=TextSubstitution(text="/dev/ttyUSB0"))

    return LaunchDescription([
        serial_port_launch_arg,
        Node(
            package='rs_passthrough',
            executable='serial_passthrough',
            parameters=[{"serial_port" : LaunchConfiguration("serial_port")}]
            ),
        Node(
            package='rs_passthrough',
            executable='joint_telemetry',
            )
            
    ])