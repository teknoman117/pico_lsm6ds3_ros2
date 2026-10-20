# SPDX-License-Identifier: Apache-2.0
"""Standalone check of the IMU: imu.launch.py with the Madgwick TF on, a fixed frame for
display and RViz. Tilting and turning the device turns the axes in RViz.

    ros2 launch pico_lsm6ds3_ros2 validation.launch.py [device:=...] [rviz:=true]

TF: world --(static, identity)--> odom --(Madgwick, orientation)--> imu_link.
RViz's fixed frame is world.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('pico_lsm6ds3_ros2')
    lc = LaunchConfiguration
    args = [
        DeclareLaunchArgument('device', default_value=''),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('stationary_source', default_value='imu'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'imu.launch.py'])),
        launch_arguments={
            'device': lc('device'),
            'frame_id': lc('frame_id'),
            'stationary_source': lc('stationary_source'),
            'publish_tf': 'true',
            'fixed_frame': 'odom',
        }.items(),
    )
    world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_odom',
        arguments=['--frame-id', 'world', '--child-frame-id', 'odom'],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'validation.rviz'])],
        condition=IfCondition(lc('rviz')),
    )
    return LaunchDescription(args + [imu, world, rviz])
