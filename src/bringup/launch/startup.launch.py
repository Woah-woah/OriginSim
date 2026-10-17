"""Start localization, maps, visualization and point cloud processing."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory('bringup')
    params_file = os.path.join(bringup_dir, 'params', 'params.yaml')

    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        parameters=[params_file, {'autostart': True}],
    )

    pointlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('point_lio_with_grid_map'),
            'launch', 'point_lio.launch.py'))
    )

    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', os.path.join(bringup_dir, 'rviz', 'default_view.rviz'),
            '--ros-args', '--log-level', 'error',
        ],
        parameters=[{'use_sim_time': True}],
        output='log',
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    modify = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('modify_map_to_odom'), 'launch', 'modify.launch.py'))
    )

    seg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('linefit_ground_segmentation_ros'),
            'launch', 'segmentation.launch.py'))
    )

    load_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('map_server'), 'launch', 'map_server.launch.py')),
        launch_arguments={
            'map': os.path.join(bringup_dir, 'map', 'rmuc_2026.yaml'),
            'use_sim_time': 'true',
            'params_file': params_file,
        }.items(),
    )

    fake_baselink = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('fake_baselink'), 'launch', 'fake_baselink.launch.py'))
    )

    return LaunchDescription([
        pointlio,
        container,
        modify,
        start_rviz,
        # seg,
        load_map_server,
        fake_baselink,
    ])
