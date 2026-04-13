from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bev_size_arg = DeclareLaunchArgument(
        'bev_size', default_value='256',
        description='BEV grid size (pixels)')
    bev_range_arg = DeclareLaunchArgument(
        'bev_range', default_value='3.0',
        description='BEV coverage range (meters, ±)')
    height_min_arg = DeclareLaunchArgument(
        'height_min', default_value='0.0',
        description='Minimum height filter (meters)')
    height_max_arg = DeclareLaunchArgument(
        'height_max', default_value='1.0',
        description='Maximum height filter (meters)')
    threshold_arg = DeclareLaunchArgument(
        'occupancy_threshold', default_value='5',
        description='Point count threshold for occupancy')
    pointcloud_arg = DeclareLaunchArgument(
        'pointcloud_topic', default_value='/camera/depth/color/points',
        description='Input point cloud topic')

    bev_mapper_node = Node(
        package='panther_slam',
        executable='bev_mapper',
        name='bev_mapper',
        parameters=[
            {'bev_size': LaunchConfiguration('bev_size')},
            {'bev_range': LaunchConfiguration('bev_range')},
            {'height_min': LaunchConfiguration('height_min')},
            {'height_max': LaunchConfiguration('height_max')},
            {'occupancy_threshold': LaunchConfiguration('occupancy_threshold')},
            {'pointcloud_topic': LaunchConfiguration('pointcloud_topic')},
        ],
        output='screen'
    )

    return LaunchDescription([
        bev_size_arg,
        bev_range_arg,
        height_min_arg,
        height_max_arg,
        threshold_arg,
        pointcloud_arg,
        bev_mapper_node,
    ])
