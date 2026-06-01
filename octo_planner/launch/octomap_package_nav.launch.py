import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    octomap_3d_share = get_package_share_directory("octomap_3d")
    octo_planner_share = get_package_share_directory("octo_planner")

    default_map_path = os.path.join(octomap_3d_share, "worlds", "garage")
    default_rviz_config = os.path.join(
        octo_planner_share, "rviz", "octomap_package_nav.rviz"
    )
    cfg_controller = os.path.join(
        octo_planner_share, "config", "nav_params.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    base_frame = LaunchConfiguration("base_frame")

    declare_use_sim = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock",
    )
    declare_map_path = DeclareLaunchArgument(
        "map_package_path",
        default_value=default_map_path,
        description="Map package directory (meta.yaml + octomap_msg.npz + layers.npz)",
    )
    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz2 config for 3D navigation",
    )
    declare_launch_rviz = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Start RViz2",
    )
    declare_launch_controller = DeclareLaunchArgument(
        "launch_controller",
        default_value="true",
        description="Start d1_controller (/planned_path -> /cmd_vel)",
    )
    declare_loc_topic = DeclareLaunchArgument(
        "localization_odom_topic",
        default_value="/localization",
        description="Robot localization nav_msgs/Odometry topic",
    )
    declare_override_child = DeclareLaunchArgument(
        "override_localization_child_frame",
        default_value="",
        description="Override TF child frame from localization message",
    )
    declare_map_frame = DeclareLaunchArgument(
        "map_frame",
        default_value="map",
        description="Fixed frame for OctoMap and planning",
    )
    declare_base_frame = DeclareLaunchArgument(
        "base_frame",
        default_value="body",
        description="Robot base frame (must match localization TF)",
    )
    declare_robot_radius = DeclareLaunchArgument(
        "robot_radius",
        default_value="0.22",
        description="Planner robot radius (m), should match map package meta.yaml",
    )

    map_package_manager_node = Node(
        package="octomap_3d",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autoload_package_path": LaunchConfiguration("map_package_path"),
                "publish_frame_id": map_frame,
                "clear_navigation_layers_on_load": True,
                "subscribe_navigation_layers": False,
                "octomap_qos_transient_local": True,
                "layer_qos_transient_local": False,
                "publish_occupied_markers": False,
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
                "use_sim_time": use_sim_time,
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "frame_id": map_frame,
            }
        ],
    )

    jie_path_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "octomap_topic": "/octomap",
                "start_topic": "/nav_bridge/start",
                "goal_topic": "/nav_bridge/goal_point",
                "goal_pose_topic": "/nav_bridge/goal_pose",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": map_frame,
                "base_frame": base_frame,
                "use_tf_for_start": True,
                "map_id": "loaded_octomap_package",
                "robot_radius": LaunchConfiguration("robot_radius"),
                "max_iterations": 500000,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "strict_direct_ground_support": True,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 2,
                "enable_preblocked_costmap": True,
                "preblocked_costmap_radius_cells": 3,
                "preblocked_costmap_weight": 1.5,
                "recompute_layers_on_octomap": False,
                "publish_layer_markers": False,
                "layer_markers_transient_local": False,
                "laser_preblocked_accumulate_frames": 5,
                "laser_preblocked_clear_on_map_import": True,
                "laser_preblocked_inflate_by_robot_radius": True,
                "publish_laser_preblocked_marker": True,
                "laser_preblocked_marker_topic": "/laser_preblocked_cells_markers",
            }
        ],
    )

    rviz_goal_nav_bridge = Node(
        package="octo_planner",
        executable="rviz_goal_nav_bridge.py",
        name="rviz_goal_nav_bridge",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": map_frame,
                "base_frame": base_frame,
                "goal_input": "clicked_point",
                "clicked_point_topic": "/clicked_point",
                "marker_topic": "/nav_bridge/selection_markers",
                "start_topic": "/nav_bridge/start",
                "goal_point_topic": "/nav_bridge/goal_point",
                "goal_pose_topic": "/nav_bridge/goal_pose",
            }
        ],
    )

    localization_tf_node = Node(
        package="octo_planner",
        executable="localization_odom_tf_broadcaster.py",
        name="localization_odom_tf_broadcaster",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "odometry_topic": LaunchConfiguration("localization_odom_topic"),
                "override_child_frame": LaunchConfiguration(
                    "override_localization_child_frame"
                ),
            }
        ],
    )

    d1_controller_node = Node(
        package="octo_planner",
        executable="d1_controller",
        name="d1_controller",
        output="screen",
        parameters=[cfg_controller, {"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("launch_controller")),
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
            declare_use_sim,
            declare_map_path,
            declare_rviz_config,
            declare_launch_rviz,
            declare_launch_controller,
            declare_loc_topic,
            declare_override_child,
            declare_map_frame,
            declare_base_frame,
            declare_robot_radius,
            map_package_manager_node,
            occupied_marker_node,
            jie_path_node,
            rviz_goal_nav_bridge,
            localization_tf_node,
            d1_controller_node,
            rviz_node,
        ]
    )
