from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    lidar_enabled = LaunchConfiguration('lidar_enabled')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    lidar_serial_baudrate = LaunchConfiguration('lidar_serial_baudrate')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')

    nodes = [
        DeclareLaunchArgument('lidar_enabled', default_value='true'),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_serial_baudrate', default_value='115200'),
        DeclareLaunchArgument('lidar_frame_id', default_value='laser'),
        Node(package='rover_vision', executable='vision_node', output='screen'),
        Node(package='rover_mission', executable='mission_node', output='screen'),
        Node(package='rover_mapping', executable='mapping_node', output='screen'),
        Node(package='rover_control1', executable='rover_driver', output='screen'),
        Node(package='rover_monitor', executable='monitor_node', output='screen'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[
                {
                    'channel_type': 'serial',
                    'serial_port': lidar_serial_port,
                    'serial_baudrate': lidar_serial_baudrate,
                    'frame_id': lidar_frame_id,
                    'inverted': False,
                    'angle_compensate': True,
                }
            ],
            output='screen',
            condition=IfCondition(lidar_enabled),
        ),
    ]

    return LaunchDescription(nodes)
