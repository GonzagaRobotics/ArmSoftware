from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arm
        Node(
            package='arm',
            executable='arm',
        )
    ])
