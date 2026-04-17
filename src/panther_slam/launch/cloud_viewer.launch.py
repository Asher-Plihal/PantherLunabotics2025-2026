from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='panther_slam',
            executable='cloud_viewer',
            name='cloud_viewer',
            output='screen',
        ),
    ])
