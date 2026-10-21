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

# 使用独立代价地图快照客户端启动执行器

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_dir = get_package_share_directory('bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    topology_file = LaunchConfiguration('topology_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    clock_params = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

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
        default_value='',
        description='Override TopoPlanner.topology_yaml; empty uses params_file')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='false',
        description='Respawn the executor if it crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Log level')

    def launch_executor(context):
        overrides = {'navigation_params_file': params_file}
        topology_override = topology_file.perform(context).strip()
        if topology_override:
            overrides['TopoPlanner.topology_yaml'] = topology_override
        return [Node(
            package='nav_executor',
            executable='nav_executor_node',
            namespace=namespace,
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[params_file, clock_params, overrides],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        )]

    load_nodes = GroupAction(actions=[OpaqueFunction(function=launch_executor)])

    # 组装启动描述
    ld = LaunchDescription()

    # 设置环境变量
    ld.add_action(stdout_linebuf_envvar)

    # 注册启动参数
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_topology_file_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # 启动导航执行器
    ld.add_action(load_nodes)

    return ld
