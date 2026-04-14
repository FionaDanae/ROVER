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

        # LIDAR
        Node(
    	    package='rplidar_ros',
    	    executable='rplidar_composition',
    	    name='rplidar_node',
    	    parameters=[{
            	'serial_port': '/dev/rover_lidar',
            	'serial_baudrate': 115200,
            	'frame_id': 'laser_frame',
            	'angle_compensate': True,
            	'auto_start': True
    	    }],
    	    output='screen'
        ),

        # TF laser -> robot
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
        ),

        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={'use_sim_time': 'false'}.items()
        ),

        # NODOS
        Node(package='rover_control1', executable='rover_driver', output='screen'),
        Node(package='rover_vision', executable='vision_node', output='screen'),
        Node(package='rover_mapping', executable='mapping_node', output='screen'),
        Node(package='rover_mission', executable='mission_node', output='screen'),

    ])
