import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    xdg_runtime_dir = os.environ.get(
        "XDG_RUNTIME_DIR", f"/tmp/runtime-{os.getuid()}"
    )
    os.makedirs(xdg_runtime_dir, mode=0o700, exist_ok=True)
    os.chmod(xdg_runtime_dir, 0o700)

    world_sdf_path = LaunchConfiguration("world_sdf_path")
    gz_config_path = LaunchConfiguration("gz_config_path")

    declare_world_sdf_path = DeclareLaunchArgument(
        "world_sdf_path",
        default_value=os.path.join(
            pkg_simulator, "resource", "worlds", "rmul_2024_world.sdf"
        ),
        description="Path to the world SDF file",
    )

    declare_gz_config_path = DeclareLaunchArgument(
        "gz_config_path",
        default_value=os.path.join(pkg_simulator, "resource", "ign", "gui.config"),
        description="Path to the Gazebo GUI configuration file",
    )

    # Set Gazebo plugin and resource path
    append_enviroment_worlds = AppendEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=os.path.join(pkg_simulator, "resource", "worlds"),
    )

    append_enviroment_models = AppendEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(pkg_simulator, "resource", "models"),
    )

    set_xdg_runtime_dir = SetEnvironmentVariable(
        name="XDG_RUNTIME_DIR",
        value=xdg_runtime_dir,
    )

    # Launch Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_version": "8",
            "gz_args": [
                world_sdf_path,
                TextSubstitution(text=" --gui-config "),
                gz_config_path,
            ],
        }.items(),
    )

    robot_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
    )

    start_velodyne_convert_tool = Node(
        package="ign_sim_pointcloud_tool",
        executable="ign_sim_pointcloud_tool_node",
        name="point_cloud_converter",
        output="screen",
        parameters=[
            {
                "pcd_topic": "livox/lidar/pointcloud",
                "n_scan": 32,
                "horizon_scan": 1875,
                "ang_bottom": 7.0,
                "ang_res_y": 1.84375,  # (7+52) / 32
            }
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_world_sdf_path)
    ld.add_action(declare_gz_config_path)
    ld.add_action(set_xdg_runtime_dir)
    ld.add_action(append_enviroment_worlds)
    ld.add_action(append_enviroment_models)
    ld.add_action(gazebo)
    ld.add_action(robot_gz_bridge)
    ld.add_action(start_velodyne_convert_tool)

    return ld
