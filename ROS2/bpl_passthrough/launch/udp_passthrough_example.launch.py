from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution , LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    ip_address_launch_arg = DeclareLaunchArgument("ip_address", default_value=TextSubstitution(text="192.168.2.4"))
    port_launch_arg = DeclareLaunchArgument("port", default_value=TextSubstitution(text="6789"))

    return LaunchDescription([
        ip_address_launch_arg,
        port_launch_arg,
        Node(
            package='bpl_passthrough',
            executable='udp_passthrough',
            parameters=[{"ip_address" : LaunchConfiguration("ip_address"),
                         "port" : LaunchConfiguration("port")}]
            ),
        Node(
            package='bpl_passthrough',
            executable='request_joint_positions',
            )
            
    ])