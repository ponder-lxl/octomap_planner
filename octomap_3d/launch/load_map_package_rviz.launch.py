import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("octomap_3d")
    default_map_path = os.path.join(pkg_share, "worlds", "garage")
    default_rviz_config = os.path.join(pkg_share, "rviz", "garage_octomap.rviz")

    map_package_path_arg = DeclareLaunchArgument(
        "map_package_path",
        default_value=default_map_path,
        description="Directory containing meta.yaml, octomap_msg.npz, layers.npz",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz2 config file",
    )
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Start RViz2",
    )

    map_package_manager_node = Node(
        package="octomap_3d",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[
            {
                "autoload_package_path": LaunchConfiguration("map_package_path"),
                "publish_frame_id": "map",
            }
        ],
    )

    occupied_marker_node = Node(
        package="octomap_3d",
        executable="octomap_to_occupied_markers_node",
        name="octomap_to_occupied_markers",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": "map",
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    return LaunchDescription(
        [
            map_package_path_arg,
            rviz_config_arg,
            launch_rviz_arg,
            map_package_manager_node,
            occupied_marker_node,
            rviz_node,
        ]
    )
