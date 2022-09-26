import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    pkg_share = get_package_share_directory('bpl_bravo_description')
    urdf_file_name = 'urdf/bravo_5_example.urdf.xacro'
    urdf_path = os.path.join(pkg_share, urdf_file_name)

    rviz_config_file = os.path.join(pkg_share, 'rviz/rviz.rviz')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Robot state publisher.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 
            'robot_description': ParameterValue(
            Command(['xacro ', str(urdf_path)]), value_type=str)}]),

        
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='RVIZ',
            arguments=['-d', rviz_config_file]
        )
    ])