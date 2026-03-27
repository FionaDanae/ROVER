from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rover_vision',
            executable='vision_node',
            output='screen'
        ),

        Node(
            package='rover_mission',
            executable='mission_node',
            output='screen'
        ),

        Node(
            package='rover_mapping',
            executable='mapping_node',
            output='screen'
        ),

        Node(
            package='rover_control1',
            executable='rover_driver',
            output='screen'
        ),

        # cuando tengas el brazo lo descomentas
        # Node(
        #     package='rover_arm',
        #     executable='arm_node',
        #     output='screen'
        # ),

    ])
