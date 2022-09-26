import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch.substitutions import TextSubstitution , LaunchConfiguration


def generate_launch_description():

    desc_pkg_share = get_package_share_directory('bpl_bravo_description')
    urdf_file_name = 'urdf/bravo_7_example.urdf.xacro'
    urdf_path = os.path.join(desc_pkg_share, urdf_file_name)

    share_directory = get_package_share_directory('bpl_bringup')
    rviz_config_file = os.path.join(share_directory, 'rviz/rviz.rviz')
    ip_address_launch_arg = DeclareLaunchArgument("ip_address", default_value=TextSubstitution(text="192.168.2.4"))
    port_launch_arg = DeclareLaunchArgument("port", default_value=TextSubstitution(text="6789"))
    return LaunchDescription([
        ip_address_launch_arg,
        port_launch_arg,
        Node(package='bpl_passthrough',
            executable='udp_passthrough',
            parameters=[{"ip_address":LaunchConfiguration("ip_address"), "port":LaunchConfiguration("port")}]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', str(urdf_path)]), value_type=str)}]),

        Node(
            package='bpl_control',
            executable='joint_state_publisher',
            parameters=[{
                "joints":[1, 2, 3, 4, 5, 6, 7],
                "joint_names":['bravo_axis_a', 'bravo_axis_b', 'bravo_axis_c', 'bravo_axis_d', 'bravo_axis_e', 'bravo_axis_f', 'bravo_axis_g'],
                'request_frequency':20,
                'publish_frequency': 20,
            }]
        ),
        Node(
            package='bpl_control',
            executable='end_effector_pose_publisher',
            parameters=[{
                "frequency":10
            }]
        ),
        
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='RVIZ',
            arguments=['-d', rviz_config_file]
        )
    ])