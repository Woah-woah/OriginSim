"""Publish static map, measured dynamic obstacles and global/local costmaps."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the static/dynamic map nodes and costmap producers."""
    bringup_dir = get_package_share_directory('bringup')

    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    clock_params = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to the static map YAML file')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'params.yaml'),
        description='Full path to the map server and costmap parameters file')

    load_nodes = GroupAction(
        actions=[
            Node(
                package='map_server',
                executable='map_server_node',
                name='map_server',
                output='screen',
                parameters=[params_file, clock_params, {'yaml_filename': map_file}]),
            Node(
                package='map_server',
                executable='dynamic_map_node',
                name='dynamic_map',
                output='screen',
                parameters=[params_file, clock_params],
                remappings=remappings),
            Node(
                package='map_server',
                executable='costmap_server_node',
                name='global_costmap',
                output='screen',
                parameters=[params_file, clock_params],
                remappings=remappings + [
                    ('costmap_raw', '/global_costmap/costmap_raw'),
                    ('costmap', '/global_costmap/costmap')]),
            Node(
                package='map_server',
                executable='costmap_server_node',
                name='local_costmap',
                output='screen',
                parameters=[params_file, clock_params],
                remappings=remappings + [
                    ('costmap_raw', '/local_costmap/costmap_raw'),
                    ('costmap', '/local_costmap/costmap')]),
        ]
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    # Start the map publisher and costmap producers
    ld.add_action(load_nodes)

    return ld
