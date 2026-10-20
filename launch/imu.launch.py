# SPDX-License-Identifier: Apache-2.0
"""The IMU driver (imu/data_raw) and Madgwick fusion (imu/data, orientation).

    ros2 launch pico_lsm6ds3_ros2 imu.launch.py [device:=/dev/input/by-id/...]
        [frame_id:=imu_link] [publish_tf:=false] [fixed_frame:=odom]
        [stationary_source:=imu|odom|both] [namespace:=]
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([FindPackageShare('pico_lsm6ds3_ros2'), 'config'])
    args = [
        DeclareLaunchArgument('device', default_value='',
                              description='evdev node of the IMU; empty: the first '
                                          '/dev/input/by-id/usb-2e8a_LSM6DS3*-event-*'),
        DeclareLaunchArgument('frame_id', default_value='imu_link'),
        DeclareLaunchArgument('publish_tf', default_value='false',
                              description='Madgwick publishes fixed_frame -> frame_id. true '
                                          'for standalone validation; false on the robot, where '
                                          'robot_localization owns TF'),
        DeclareLaunchArgument('fixed_frame', default_value='odom',
                              description='parent frame of the Madgwick TF (publish_tf)'),
        DeclareLaunchArgument('stationary_source', default_value='imu',
                              description='what decides "at rest" for the gyro bias: imu, '
                                          'odom (the wheel odometry) or both'),
        DeclareLaunchArgument('driver_params', default_value=PathJoinSubstitution(
            [config, 'driver.yaml'])),
        DeclareLaunchArgument('madgwick_params', default_value=PathJoinSubstitution(
            [config, 'madgwick.yaml'])),
        DeclareLaunchArgument('namespace', default_value=''),
    ]
    lc = LaunchConfiguration

    driver = Node(
        package='pico_lsm6ds3_ros2',
        executable='driver',
        name='pico_lsm6ds3_driver',
        output='screen',
        parameters=[
            lc('driver_params'),
            {
                'device': ParameterValue(lc('device'), value_type=str),
                'frame_id': lc('frame_id'),
                'stationary_source': lc('stationary_source'),
            },
        ],
    )
    fusion = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[
            lc('madgwick_params'),
            {
                'publish_tf': ParameterValue(lc('publish_tf'), value_type=bool),
                'fixed_frame': lc('fixed_frame'),
            },
        ],
    )
    return LaunchDescription(args + [
        GroupAction([PushRosNamespace(lc('namespace')), driver, fusion]),
    ])
