"""Load the configured static transform into the bringup container."""

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():

    load_all_tf = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='tf2_ros',
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                name='map_to_odom',
                parameters=[{
                    'translation.x': 0.0,
                    'translation.y': 0.0,
                    'translation.z': 0.0,
                    'rotation.x': 0.0,
                    'rotation.y': 0.0,
                    'rotation.z': 0.521,
                    'rotation.w': 0.854,
                    'frame_id': 'map',
                    'child_frame_id': 'odom'
                }]
            ),
        ]
    )

    return LaunchDescription([load_all_tf])
