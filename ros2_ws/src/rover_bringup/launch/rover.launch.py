from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    slam_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    return LaunchDescription([
        Node(package='rover_vision', executable='vision_node', output='screen'),
        Node(package='rover_mission', executable='mission_node', output='screen'),
        Node(package='rover_mapping', executable='mapping_node', output='screen'),
        Node(package='rover_control1', executable='rover_driver', output='screen'),
        Node(package='rover_monitor', executable='monitor_node', output='screen'),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         'serial_port': '/dev/ttyUSB0', 
                         'serial_baudrate': 115200,
                         'frame_id': 'laser_frame',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'
        ),

        # Transformación estática (Ajusta la altura en Z, aquí puse 0.2 metros)
        # Define que el 'laser' está pegado al 'base_link' (centro del robot)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={'use_sim_time': 'false'}.items()
        )
    ])
