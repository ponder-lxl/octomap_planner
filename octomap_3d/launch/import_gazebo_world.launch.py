import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def build_nodes(context):
    world_file = LaunchConfiguration("world_file").perform(context).strip()
    world_name = LaunchConfiguration("world_name").perform(context).strip()
    world_correction_roll = float(
        LaunchConfiguration("world_correction_roll").perform(context) or "1.57"
    )
    world_correction_pitch = float(
        LaunchConfiguration("world_correction_pitch").perform(context) or "0.0"
    )
    world_correction_yaw = float(
        LaunchConfiguration("world_correction_yaw").perform(context) or "0.0"
    )
    if not world_file and world_name:
        world_file = os.path.join(
            get_package_share_directory("octomap_3d"), "worlds", world_name
        )

    extra_model_paths = []
    home_gazebo_models = os.path.join(os.path.expanduser("~"), ".gazebo", "models")
    if os.path.isdir(home_gazebo_models):
        extra_model_paths.append(home_gazebo_models)
    try:
        tb3_share = get_package_share_directory("turtlebot3_gazebo")
        tb3_mesh = os.path.join(tb3_share, "mesh")
        if os.path.isdir(tb3_mesh):
            extra_model_paths.append(tb3_mesh)
    except Exception:
        pass

    world_to_octomap_node = Node(
        package="octomap_3d",
        executable="world_to_octomap_node",
        name="world_to_octomap",
        output="screen",
        parameters=[
            {
                "world_file": world_file,
                "resolution": 0.2,
                "xy_window_size_m": 150.0,
                "frame_id": "map",
                "octomap_topic": "/octomap",
                "marker_topic": "/octomap_occupied_markers",
                "gazebo_model_paths": extra_model_paths,
                "world_correction_roll": world_correction_roll,
                "world_correction_pitch": world_correction_pitch,
                "world_correction_yaw": world_correction_yaw,
            }
        ],
    )

    planner_node = Node(
        package="octo_planner",
        executable="jie_path_node",
        name="jie_path_node",
        output="screen",
        parameters=[
            {
                "octomap_topic": "/octomap",
                "start_topic": "/start_point",
                "goal_topic": "/goal_point",
                "path_topic": "/planned_path",
                "path_marker_topic": "/planned_path_marker",
                "preblocked_marker_topic": "/preblocked_cells_markers",
                "traversable_marker_topic": "/traversable_cells_markers",
                "risk_cost_topic": "/risk_cost_cells",
                "frame_id": "map",
                "map_id": "world_jie_path_map",
                "source_world_file": world_file,
                "robot_radius": 0.12,
                "max_iterations": 500000,
                "snap_search_radius_cells": 12,
                "require_ground_support": True,
                "ground_support_xy_radius_cells": 1,
                "ground_support_depth_cells": 2,
            }
        ],
    )

    world_selector_gui_node = Node(
        package="octomap_3d",
        executable="world_selector_gui.py",
        name="world_selector_gui",
        output="screen",
        parameters=[
            {
                "initial_world_file": world_file,
                "world_file_cmd_topic": "/world_file_cmd",
                "occupied_marker_topic": "/octomap_occupied_markers",
                "preblocked_topic": "/preblocked_cells_markers",
                "traversable_topic": "/traversable_cells_markers",
            }
        ],
    )

    map_package_manager_node = Node(
        package="octomap_3d",
        executable="map_package_manager",
        name="map_package_manager",
        output="screen",
        parameters=[
            {
                "save_skip_export_if_ready": True,
                "save_recompute_layers": False,
                "save_export_timeout_sec": 600.0,
                "save_layer_sync_timeout_sec": 600.0,
                "save_meta_timeout_sec": 120.0,
                "planner_map_id": "world_jie_path_map",
                "planner_source_world_file": world_file,
                "planner_robot_radius": 0.12,
                "planner_snap_search_radius_cells": 12,
                "planner_require_ground_support": True,
                "planner_strict_direct_ground_support": True,
                "planner_ground_support_xy_radius_cells": 1,
                "planner_ground_support_depth_cells": 2,
            }
        ],
        condition=IfCondition(LaunchConfiguration("launch_map_gui")),
    )

    return [
        world_selector_gui_node,
        world_to_octomap_node,
        planner_node,
        map_package_manager_node,
    ]


def generate_launch_description():
    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value="",
        description="Absolute path to Gazebo .world/.sdf file. Overrides world_name.",
    )
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="",
        description="World filename under the octomap_3d package worlds directory.",
    )
    launch_map_gui_arg = DeclareLaunchArgument(
        "launch_map_gui",
        default_value="true",
        description="Launch map package manager and PyQt save/load window",
    )
    world_correction_roll_arg = DeclareLaunchArgument(
        "world_correction_roll",
        default_value="1.57",
        description="Roll correction (rad) applied to world geometry before OctoMap export.",
    )
    world_correction_pitch_arg = DeclareLaunchArgument(
        "world_correction_pitch",
        default_value="0.0",
        description="Pitch correction (rad) applied to world geometry before OctoMap export.",
    )
    world_correction_yaw_arg = DeclareLaunchArgument(
        "world_correction_yaw",
        default_value="0.0",
        description="Yaw correction (rad) applied to world geometry before OctoMap export.",
    )

    return LaunchDescription(
        [
            world_file_arg,
            world_name_arg,
            launch_map_gui_arg,
            world_correction_roll_arg,
            world_correction_pitch_arg,
            world_correction_yaw_arg,
            OpaqueFunction(function=build_nodes),
        ]
    )
