# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch standalone navigation using independent map_server costmaps."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # Get the launch directories
    nav_executor_dir = get_package_share_directory('nav_executor')
    bringup_dir = get_package_share_directory('bringup')
    planner_dir = get_package_share_directory('topology_global_planner')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')
    topology_file = LaunchConfiguration('topology_file')
    use_respawn = LaunchConfiguration('use_respawn')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'params.yaml'),
        description='Full path to the navigation parameters file')

    declare_topology_file_cmd = DeclareLaunchArgument(
        'topology_file',
        default_value=os.path.join(planner_dir, 'config', 'topo.yaml'),
        description='Full path to the topology map YAML file')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Respawn the executor if it crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level')

    bringup_cmd_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_executor_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'topology_file': topology_file,
                    'use_respawn': use_respawn,
                    'log_level': log_level,
                }.items()),
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_topology_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Include the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
