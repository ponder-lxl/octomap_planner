"""
从 PCD 点云离线生成 OctoMap 地图包（occupied + preblocked + traversable + risk）。

配置：编辑 share/octomap_3d/config/build_map_package_from_pcd.yaml（pcd_file、package_output_dir 等）。

启动节点（可由 yaml 中 launch.* 开关）：
  - pcd_to_octomap_node
  - jie_path_node（在线重算规划图层）
  - octomap_to_occupied_markers_node（可选）
  - map_package_manager
  - map_save_gui（可选）
  - rviz2（可选）

流程：
  1. pcd_to_octomap_node 读取 PCD -> /octomap
  2. jie_path_node 在线重算三层（大场景可能较久）
  3. 终端出现 Navigation layers generation COMPLETE 后保存：
     - 地图保存 GUI（默认启动），或
     - ros2 service call /map_package_manager/save_package map_3d_msgs/srv/SaveNavigationMapPackage \\
         "{package_path: '<package_output_dir>', overwrite: true}"

示例：
  ros2 launch octomap_3d build_map_package_from_pcd.launch.py
  ros2 launch octomap_3d build_map_package_from_pcd.launch.py config_file:=/path/to/my.yaml
"""
from __future__ import annotations

import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _load_yaml(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    return data if isinstance(data, dict) else {}


def _build_nodes(context):
    pkg_share = get_package_share_directory("octomap_3d")
    config_path = LaunchConfiguration("config_file").perform(context).strip()
    if not config_path:
        config_path = os.path.join(pkg_share, "config", "build_map_package_from_pcd.yaml")

    cfg = _load_yaml(config_path)
    launch_cfg = cfg.get("launch", {}) if isinstance(cfg.get("launch"), dict) else {}
    pcd_cfg = cfg.get("pcd_to_octomap", {}) if isinstance(cfg.get("pcd_to_octomap"), dict) else {}
    jie_cfg = cfg.get("jie_path_node", {}) if isinstance(cfg.get("jie_path_node"), dict) else {}
    mgr_cfg = cfg.get("map_package_manager", {}) if isinstance(cfg.get("map_package_manager"), dict) else {}

    pcd_file = str(cfg.get("pcd_file", "")).strip()
    package_output_dir = str(cfg.get("package_output_dir", "")).strip()
    map_id = str(cfg.get("map_id", jie_cfg.get("map_id", "pcd_export_map"))).strip()
    frame_id = str(cfg.get("frame_id", "map")).strip()
    use_sim_time = _as_bool(cfg.get("use_sim_time"), False)

    if not pcd_file and not str(pcd_cfg.get("map_cloud_topic", "")).strip():
        raise RuntimeError(
            f"config {config_path}: set pcd_file or pcd_to_octomap.map_cloud_topic"
        )

    rviz_config = str(launch_cfg.get("rviz_config", "")).strip()
    if not rviz_config:
        rviz_config = os.path.join(pkg_share, "rviz", "build_map_package_from_pcd.rviz")

    launch_rviz = _as_bool(launch_cfg.get("launch_rviz"), True)
    launch_map_save_gui = _as_bool(launch_cfg.get("launch_map_save_gui"), True)
    launch_occupied_markers = _as_bool(launch_cfg.get("launch_occupied_markers"), True)

    print(
        f"[build_map_package_from_pcd] config={config_path}\n"
        f"  pcd_file={pcd_file or '(from map_cloud_topic)'}\n"
        f"  package_output_dir={package_output_dir or '(set in save GUI)'}\n"
        f"  map_id={map_id} frame_id={frame_id} use_sim_time={use_sim_time}\n"
        f"  After 'Navigation layers generation COMPLETE', save to package_output_dir."
    )

    pcd_to_octomap_node = Node(
        package="octomap_3d",
        executable="pcd_to_octomap_node",
        name="pcd_to_octomap",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "pcd_file": pcd_file,
                "pcd_file_cmd_topic": str(pcd_cfg.get("pcd_file_cmd_topic", "/pcd_file_cmd")),
                "map_cloud_topic": str(pcd_cfg.get("map_cloud_topic", "")),
                "octomap_topic": str(pcd_cfg.get("octomap_topic", "/octomap")),
                "frame_id": frame_id,
                "resolution": float(pcd_cfg.get("resolution", 0.2)),
                "voxel_downsample_m": float(pcd_cfg.get("voxel_downsample_m", 0.1)),
                "min_points_per_voxel": int(pcd_cfg.get("min_points_per_voxel", 2)),
                "min_cluster_voxels": int(pcd_cfg.get("min_cluster_voxels", 2)),
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
                "octomap_topic": str(jie_cfg.get("octomap_topic", "/octomap")),
                "preblocked_marker_topic": str(
                    jie_cfg.get("preblocked_marker_topic", "/preblocked_cells_markers")
                ),
                "traversable_marker_topic": str(
                    jie_cfg.get("traversable_marker_topic", "/traversable_cells_markers")
                ),
                "risk_cost_topic": str(jie_cfg.get("risk_cost_topic", "/risk_cost_cells")),
                "frame_id": frame_id,
                "map_id": map_id,
                "source_world_file": "",
                "recompute_layers_on_octomap": _as_bool(
                    jie_cfg.get("recompute_layers_on_octomap"), True
                ),
                "robot_radius": float(jie_cfg.get("robot_radius", 0.22)),
                "max_iterations": int(jie_cfg.get("max_iterations", 500000)),
                "snap_search_radius_cells": int(
                    jie_cfg.get("snap_search_radius_cells", 12)
                ),
                "require_ground_support": _as_bool(
                    jie_cfg.get("require_ground_support"), True
                ),
                "strict_direct_ground_support": _as_bool(
                    jie_cfg.get("strict_direct_ground_support"), False
                ),
                "ground_support_xy_radius_cells": int(
                    jie_cfg.get("ground_support_xy_radius_cells", 1)
                ),
                "ground_support_depth_cells": int(
                    jie_cfg.get("ground_support_depth_cells", 1)
                ),
                "lowest_traversable_only": _as_bool(
                    jie_cfg.get("lowest_traversable_only"), False
                ),
                "enable_preblocked_costmap": _as_bool(
                    jie_cfg.get("enable_preblocked_costmap"), True
                ),
                "preblocked_costmap_radius_cells": int(
                    jie_cfg.get("preblocked_costmap_radius_cells", 3)
                ),
                "preblocked_costmap_weight": float(
                    jie_cfg.get("preblocked_costmap_weight", 2.5)
                ),
                "layer_markers_transient_local": _as_bool(
                    jie_cfg.get("layer_markers_transient_local"), True
                ),
            }
        ],
    )

    nodes = [pcd_to_octomap_node, jie_path_node]

    if launch_occupied_markers:
        nodes.append(
            Node(
                package="octomap_3d",
                executable="octomap_to_occupied_markers_node",
                name="octomap_to_occupied_markers",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "octomap_topic": str(pcd_cfg.get("octomap_topic", "/octomap")),
                        "marker_topic": "/octomap_occupied_markers",
                        "frame_id": frame_id,
                    }
                ],
            )
        )

    mgr_params = {
        "use_sim_time": use_sim_time,
        "publish_frame_id": frame_id,
        "planner_map_id": str(mgr_cfg.get("planner_map_id", map_id)),
        "publish_navigation_layers": _as_bool(
            mgr_cfg.get("publish_navigation_layers"), True
        ),
        "layer_qos_transient_local": _as_bool(
            mgr_cfg.get("layer_qos_transient_local"), True
        ),
        "save_skip_export_if_ready": _as_bool(mgr_cfg.get("save_skip_export_if_ready"), True),
        "save_recompute_layers": _as_bool(mgr_cfg.get("save_recompute_layers"), False),
        "save_export_timeout_sec": float(mgr_cfg.get("save_export_timeout_sec", 7200.0)),
        "save_layer_sync_timeout_sec": float(
            mgr_cfg.get("save_layer_sync_timeout_sec", 7200.0)
        ),
        "save_meta_timeout_sec": float(mgr_cfg.get("save_meta_timeout_sec", 300.0)),
    }
    nodes.append(
        Node(
            package="octomap_3d",
            executable="map_package_manager",
            name="map_package_manager",
            output="screen",
            parameters=[mgr_params],
        )
    )

    if launch_map_save_gui:
        nodes.append(
            Node(
                package="octomap_3d",
                executable="map_save_gui",
                name="map_save_gui",
                output="screen",
            )
        )

    if launch_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("octomap_3d")
    default_config = os.path.join(pkg_share, "config", "build_map_package_from_pcd.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="YAML 配置（pcd_file、package_output_dir、规划参数等）",
            ),
            OpaqueFunction(function=_build_nodes),
        ]
    )
