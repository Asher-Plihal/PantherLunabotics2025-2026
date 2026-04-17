from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py'
        ]),
        launch_arguments={
            'enable_gyro':          'true',
            'enable_accel':         'true',
            'unite_imu_method':     '1',
            'align_depth.enable':   'true',
            'enable_sync':          'true',
            'rgb_camera.profile':   '640x480x30',
            'depth_module.profile': '640x480x30',
        }.items()
    )

    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick_node',
        parameters=[{
            'use_mag':     False,
            'publish_tf':  False,
            'world_frame': 'enu',
        }],
        remappings=[
            ('/imu/data_raw', '/camera/camera/imu'),
        ]
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('rtabmap_launch'), 'launch'),
            '/rtabmap.launch.py'
        ]),
        launch_arguments={
            'rtabmap_args':       '--delete_db_on_start --Optimizer/GravitySigma 0.3 --Grid/3D true',
            'odom_args':          '--Vis/MinInliers 10 --Vis/MaxDepth 4.0 --OdomF2M/MaxSize 1000 --Kp/MaxFeatures 500',
            'rgb_topic':          '/camera/camera/color/image_raw',
            'depth_topic':        '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic':  '/camera/camera/color/camera_info',
            'approx_sync':        'true',
            'wait_imu_to_init':   'false',
            'frame_id':           'camera_link',
            'rviz':               'false',  # disable on robot, enable on laptop
        }.items()
    )

    # Delay planner start by 5s to give camera + rtabmap time to initialise
    slam_planner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='panther_slam',
                executable='slam_planner',
                name='slam_planner',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        realsense_launch,
        imu_filter,
        rtabmap_launch,
        slam_planner,
    ])