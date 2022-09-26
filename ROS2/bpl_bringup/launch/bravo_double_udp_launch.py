import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue
from launch.substitutions import TextSubstitution , LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction

def generate_launch_description():

    desc_pkg_share = get_package_share_directory('bpl_bravo_description')
    urdf_file_name = 'urdf/bravo_double_example.urdf.xacro'
    urdf_path = os.path.join(desc_pkg_share, urdf_file_name)

    share_directory = get_package_share_directory('bpl_bringup')
    rviz_config_file = os.path.join(share_directory, 'rviz/rviz_double.rviz')
    ip_address_a_launch_arg = DeclareLaunchArgument("ip_address_a", default_value=TextSubstitution(text="192.168.2.7"))
    port_a_launch_arg = DeclareLaunchArgument("port_a", default_value=TextSubstitution(text="6789"))

    ip_address_b_launch_arg = DeclareLaunchArgument("ip_address_b", default_value=TextSubstitution(text="192.168.2.4"))
    port_b_launch_arg = DeclareLaunchArgument("port_b", default_value=TextSubstitution(text="6789"))


    bravo_a_launch_description = LaunchDescription([
        Node(
            package='bpl_control',
            executable='joint_state_publisher',
            parameters=[{
                "joints":[1, 2, 3, 4, 5, 6, 7],
                "joint_names":['bravo_a_axis_a', 'bravo_a_axis_b', 'bravo_a_axis_c', 'bravo_a_axis_d', 'bravo_a_axis_e', 'bravo_a_axis_f', 'bravo_a_axis_g'],
                'request_frequency':20,
                'publish_frequency': 20,
            }],
        ),      
        Node(package='bpl_passthrough',
            executable='udp_passthrough',
            parameters=[{"ip_address":LaunchConfiguration("ip_address_a"), "port": LaunchConfiguration("port_a")}]),
        Node(
            package='bpl_control',
            executable='end_effector_pose_publisher',
            parameters=[{
                "frequency":10,
                "frame_id":"bravo_a_base_link"
            }]
        )
    ])
    bravo_a_launch_description_with_ns = GroupAction(
     actions=[
         PushRosNamespace('bravo_a'),
         bravo_a_launch_description,
      ])

    bravo_b_launch_description = LaunchDescription([
        Node(
            package='bpl_control',
            executable='joint_state_publisher',
            parameters=[{
                "joints":[1, 2, 3, 4, 5],
                "joint_names":['bravo_b_axis_a', 'bravo_b_axis_b', 'bravo_b_axis_c', 'bravo_b_axis_d', 'bravo_b_axis_e'],
                'request_frequency':20,
                'publish_frequency': 20,
            }],
        ),   
        Node(package='bpl_passthrough',
            executable='udp_passthrough',
            parameters=[{"ip_address":LaunchConfiguration("ip_address_b"), "port": LaunchConfiguration("port_b")}],),
        Node(
            package='bpl_control',
            executable='end_effector_pose_publisher',
            parameters=[{
                "frequency":10,
                "frame_id":"bravo_b_base_link"
            }]
        )
    ])

    bravo_b_launch_description_with_ns = GroupAction(
     actions=[
         PushRosNamespace('bravo_b'),
         bravo_b_launch_description,
      ])

    return LaunchDescription([
        ip_address_a_launch_arg,
        port_a_launch_arg,
        ip_address_b_launch_arg,
        port_b_launch_arg,
        bravo_a_launch_description_with_ns,
        bravo_b_launch_description_with_ns,

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            parameters=[{"source_list": ["bravo_b/joint_states", "bravo_a/joint_states"
            ]}]),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': ParameterValue(
            Command(['xacro ', str(urdf_path)]), value_type=str)}]),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='RVIZ',
            arguments=['-d', rviz_config_file],
        )
    ])