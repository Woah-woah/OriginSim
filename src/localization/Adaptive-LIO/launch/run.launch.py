from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # 定义参数
    project = "adaptive_lio"
    adaptive_lio_cfg_dir = LaunchConfiguration("adaptive_lio_cfg_dir")

    adaptive_lio_dir = get_package_share_directory("adaptive_lio")
    declare_adaptive_lio_cfg_dir = DeclareLaunchArgument(
        "adaptive_lio_cfg_dir",
        default_value=PathJoinSubstitution([adaptive_lio_dir, "config", "mapping_m.yaml"]),
        description="Path to the adaptive_lio config file",
    )
    rviz = LaunchConfiguration('rviz', default='true')

    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Set "true" to launch rviz')

    # Adaptive LIO 节点
    adaptive_lio_node = Node(
        package=project,
        executable=project,
        name=project,
        output='screen',
        arguments=['--config_file', adaptive_lio_cfg_dir],
        # 与仿真传感器和导航共用 /clock。
        parameters=[{'use_sim_time': True}],
    )

    # RVIZ 可视化节点
    
    rviz_group = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
           # arguments=['-d', '/home/sb/adaptive-lio/src/Adaptive-LIO/launch/rviz.rviz'],
            condition=IfCondition(rviz)
        )
    


    ld = LaunchDescription(
        [
            declare_rviz_cmd,
            declare_adaptive_lio_cfg_dir,
            adaptive_lio_node,
            # rviz_group,
        ]
    )


    return ld
