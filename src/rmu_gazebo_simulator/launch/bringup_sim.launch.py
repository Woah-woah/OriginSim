import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")

    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        
    if "uc" in selected_world :
        world_type = 1
    elif "ul" in selected_world:
        world_type = 4
    else: world_type = 0

    world_sdf_path = os.path.join(
        pkg_simulator, "resource", "worlds", f"{selected_world}_world.sdf"
    )
    gz_config_path = os.path.join(pkg_simulator, "resource", "ign", "gui.config")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world_sdf_path": world_sdf_path,
            "gz_config_path": gz_config_path,
        }.items(),
    )

    spawn_robots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulator, "launch", "spawn_robots.launch.py")
        ),
        launch_arguments={
            "gz_world_path": gz_world_path,
            "world": selected_world,
        }.items(),
    )


    referee_system = Node(
            package="referee_pub",
            executable="referee_pub",
            parameters=[
                {
                    "game_type": world_type,
                    "game_progress": 4,
                    # "stage_remain_time": 419,
                    "red_1_hp": 500,
                    "red_2_hp": 250,
                    "red_3_hp": 400,
                    "red_4_hp": 400,
                    "red_5_hp": 400,
                    "red_7_hp": 600,
                    "blue_1_hp": 500,
                    "blue_2_hp": 250,
                    "blue_3_hp": 400,
                    "blue_4_hp": 400,
                    "blue_5_hp": 400,
                    "blue_7_hp": 600,
                    # "rfid_status": 0,
                }
            ],
        )
        
    ld = LaunchDescription()

    ld.add_action(gazebo_launch)
    ld.add_action(spawn_robots_launch)
    ld.add_action(referee_system)

    return ld
