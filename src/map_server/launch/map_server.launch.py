"""Start the single MapServer producer."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    def create_node(context):
        overrides = {
            'yaml_filename': LaunchConfiguration('map').perform(context),
            'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
        }
        for name in ('static_pcd', 'region_yaml'):
            value = LaunchConfiguration(name).perform(context).strip()
            if value:
                overrides[name] = value
        return [Node(package='map_server', executable='map_server_node', name='map_server',
                     output='screen', parameters=[LaunchConfiguration('params_file'), overrides])]

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute occupancy map YAML'),
        DeclareLaunchArgument('static_pcd', default_value='', description='Override reference PCD; empty uses params_file'),
        DeclareLaunchArgument('region_yaml', default_value='', description='Override aligned Region layer; empty uses params_file'),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('bringup'), 'params', 'params.yaml')),
        OpaqueFunction(function=create_node),
    ])
