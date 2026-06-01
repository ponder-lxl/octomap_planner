#!/usr/bin/env python3

from __future__ import annotations

import copy
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Optional

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import Point
from map_3d_msgs.srv import (
    ExportNavigationSnapshot,
    GetNavigationMapMeta,
    LoadNavigationMapPackage,
    SaveNavigationMapPackage,
)
from octomap_msgs.msg import Octomap
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class MapPackageManager(Node):
    def __init__(self) -> None:
        super().__init__("map_package_manager")

        self.declare_parameter("octomap_topic", "/octomap")
        self.declare_parameter("occupied_marker_topic", "/octomap_occupied_markers")
        self.declare_parameter("preblocked_topic", "/preblocked_cells_markers")
        self.declare_parameter("traversable_topic", "/traversable_cells_markers")
        self.declare_parameter("risk_cost_topic", "/risk_cost_cells")
        self.declare_parameter("planner_meta_service", "/jie_path_node/get_meta")
        self.declare_parameter("planner_export_service", "/jie_path_node/export_snapshot")
        self.declare_parameter("autoload_package_path", "")
        self.declare_parameter("save_export_timeout_sec", 600.0)
        self.declare_parameter("save_recompute_layers", False)
        self.declare_parameter("save_skip_export_if_ready", True)
        self.declare_parameter("save_layer_sync_timeout_sec", 600.0)
        self.declare_parameter("save_meta_timeout_sec", 120.0)
        self.declare_parameter("save_progress_topic", "/map_package_manager/save_progress")
        self.declare_parameter("planner_map_id", "world_jie_path_map")
        self.declare_parameter("planner_source_world_file", "")
        self.declare_parameter("planner_robot_radius", 0.12)
        self.declare_parameter("planner_snap_search_radius_cells", 12)
        self.declare_parameter("planner_require_ground_support", True)
        self.declare_parameter("planner_strict_direct_ground_support", True)
        self.declare_parameter("planner_ground_support_xy_radius_cells", 1)
        self.declare_parameter("planner_ground_support_depth_cells", 2)
        self.declare_parameter("planner_enable_preblocked_costmap", True)
        self.declare_parameter("planner_preblocked_costmap_radius_cells", 3)
        self.declare_parameter("planner_preblocked_costmap_weight", 1.5)
        self.declare_parameter("publish_frame_id", "map")
        self.declare_parameter("publish_navigation_layers", True)
        # OctoMap 与规划图层分开 QoS：OctoMap 用 latched 供 jie_path/occupied 节点；图层用 volatile 避免 RViz 叠影
        self.declare_parameter("octomap_qos_transient_local", True)
        self.declare_parameter("layer_qos_transient_local", True)
        self.declare_parameter("publish_occupied_markers", True)
        self.declare_parameter("clear_navigation_layers_on_load", True)
        # 仅发布地图包、不监听图层话题时设为 false，避免订阅到 DDS 缓存的旧 transient_local 图层
        self.declare_parameter("subscribe_navigation_layers", True)

        self._publish_navigation_layers = bool(
            self.get_parameter("publish_navigation_layers").value
        )
        self._octomap_qos_transient_local = bool(
            self.get_parameter("octomap_qos_transient_local").value
        )
        self._layer_qos_transient_local = bool(
            self.get_parameter("layer_qos_transient_local").value
        )
        self._publish_occupied_markers = bool(
            self.get_parameter("publish_occupied_markers").value
        )
        self._subscribe_navigation_layers = bool(
            self.get_parameter("subscribe_navigation_layers").value
        )
        octomap_qos = self._make_layer_qos(self._octomap_qos_transient_local)
        layer_qos = self._make_layer_qos(self._layer_qos_transient_local)
        latched_qos = self._make_layer_qos(True)
        callback_group = ReentrantCallbackGroup()

        self._latest_octomap: Optional[Octomap] = None
        self._latest_occupied: Optional[Marker] = None
        self._latest_preblocked: Optional[Marker] = None
        self._latest_traversable: Optional[Marker] = None
        self._latest_risk_cost: Optional[PointCloud2] = None

        if self._subscribe_navigation_layers:
            self.create_subscription(
                Octomap,
                self.get_parameter("octomap_topic").value,
                self._on_octomap,
                octomap_qos,
                callback_group=callback_group,
            )
            self.create_subscription(
                Marker,
                self.get_parameter("occupied_marker_topic").value,
                self._on_occupied,
                layer_qos,
                callback_group=callback_group,
            )
            self.create_subscription(
                Marker,
                self.get_parameter("preblocked_topic").value,
                self._on_preblocked,
                layer_qos,
                callback_group=callback_group,
            )
            self.create_subscription(
                Marker,
                self.get_parameter("traversable_topic").value,
                self._on_traversable,
                layer_qos,
                callback_group=callback_group,
            )
            self.create_subscription(
                PointCloud2,
                self.get_parameter("risk_cost_topic").value,
                self._on_risk_cost,
                layer_qos,
                callback_group=callback_group,
            )

        self.octomap_pub = None
        self.occupied_pub = None
        self.preblocked_pub = None
        self.traversable_pub = None
        self.risk_cost_pub = None
        if self._publish_navigation_layers:
            self._ensure_layer_publishers(octomap_qos, layer_qos)

        self._save_progress_pub = self.create_publisher(
            String, self.get_parameter("save_progress_topic").value, latched_qos
        )

        self.meta_client = self.create_client(
            GetNavigationMapMeta,
            self.get_parameter("planner_meta_service").value,
            callback_group=callback_group,
        )
        self.export_client = self.create_client(
            ExportNavigationSnapshot,
            self.get_parameter("planner_export_service").value,
            callback_group=callback_group,
        )

        self.create_service(
            SaveNavigationMapPackage,
            "~/save_package",
            self._handle_save_package,
            callback_group=callback_group,
        )
        self.create_service(
            LoadNavigationMapPackage,
            "~/load_package",
            self._handle_load_package,
            callback_group=callback_group,
        )
        self._autoload_timer = self.create_timer(1.0, self._autoload_package_once)

        self.get_logger().info(
            "map_package_manager started. save_service=~/save_package load_service=~/load_package "
            f"publish_navigation_layers={self._publish_navigation_layers} "
            f"subscribe_navigation_layers={self._subscribe_navigation_layers} "
            f"octomap_qos_transient_local={self._octomap_qos_transient_local} "
            f"layer_qos_transient_local={self._layer_qos_transient_local} "
            f"publish_occupied_markers={self._publish_occupied_markers}"
        )

    @staticmethod
    def _make_layer_qos(transient_local: bool) -> QoSProfile:
        durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        )
        depth = 1 if transient_local else 10
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=durability,
        )

    def _ensure_layer_publishers(
        self,
        octomap_qos: Optional[QoSProfile] = None,
        layer_qos: Optional[QoSProfile] = None,
    ) -> None:
        if self.octomap_pub is not None:
            return
        if octomap_qos is None:
            octomap_qos = self._make_layer_qos(self._octomap_qos_transient_local)
        if layer_qos is None:
            layer_qos = self._make_layer_qos(self._layer_qos_transient_local)
        self.octomap_pub = self.create_publisher(
            Octomap, self.get_parameter("octomap_topic").value, octomap_qos
        )
        if self._publish_occupied_markers:
            self.occupied_pub = self.create_publisher(
                Marker, self.get_parameter("occupied_marker_topic").value, layer_qos
            )
        self.preblocked_pub = self.create_publisher(
            Marker, self.get_parameter("preblocked_topic").value, layer_qos
        )
        self.traversable_pub = self.create_publisher(
            Marker, self.get_parameter("traversable_topic").value, layer_qos
        )
        self.risk_cost_pub = self.create_publisher(
            PointCloud2, self.get_parameter("risk_cost_topic").value, layer_qos
        )

    @staticmethod
    def _is_marker_cleared(msg: Marker) -> bool:
        return msg.action == Marker.DELETEALL or (
            msg.type == Marker.CUBE_LIST and not msg.points
        )

    def _publish_marker_deleteall(
        self, publisher, namespace: str, frame_id: str
    ) -> None:
        if publisher is None:
            return
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.ns = namespace
        msg.action = Marker.DELETEALL
        publisher.publish(msg)

    def _publish_empty_cube_list(
        self,
        publisher,
        namespace: str,
        frame_id: str,
        scale: tuple[float, float, float] = (0.2, 0.2, 0.2),
    ) -> None:
        if publisher is None:
            return
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.ns = namespace
        msg.id = 0
        msg.type = Marker.CUBE_LIST
        msg.action = Marker.ADD
        msg.pose.orientation.w = 1.0
        msg.scale.x, msg.scale.y, msg.scale.z = scale
        msg.color.a = 0.0
        publisher.publish(msg)

    def _clear_navigation_layers_in_rviz(self, frame_id: str) -> None:
        """加载新地图包前先清除 RViz 中旧图层（DELETEALL + 空 CUBE_LIST）。"""
        octomap_qos = self._make_layer_qos(self._octomap_qos_transient_local)
        layer_qos = self._make_layer_qos(self._layer_qos_transient_local)
        self._ensure_layer_publishers(octomap_qos, layer_qos)
        self._publish_marker_deleteall(self.preblocked_pub, "preblocked_cells", frame_id)
        self._publish_marker_deleteall(self.traversable_pub, "traversable_cells", frame_id)
        if self.occupied_pub is not None:
            self._publish_marker_deleteall(self.occupied_pub, "occupied_voxels", frame_id)
        self._publish_empty_cube_list(self.preblocked_pub, "preblocked_cells", frame_id)
        self._publish_empty_cube_list(self.traversable_pub, "traversable_cells", frame_id)
        if self.risk_cost_pub is not None:
            header = PointCloud2()
            header.header.stamp = self.get_clock().now().to_msg()
            header.header.frame_id = frame_id
            empty_risk = point_cloud2.create_cloud(
                header.header,
                [
                    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(
                        name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
                    ),
                ],
                [],
            )
            self.risk_cost_pub.publish(empty_risk)

    def _on_octomap(self, msg: Octomap) -> None:
        self._latest_octomap = copy.deepcopy(msg)

    def _on_occupied(self, msg: Marker) -> None:
        if self._is_marker_cleared(msg):
            self._latest_occupied = None
            return
        if msg.type == Marker.CUBE_LIST:
            self._latest_occupied = copy.deepcopy(msg)

    def _on_preblocked(self, msg: Marker) -> None:
        if self._is_marker_cleared(msg):
            self._latest_preblocked = None
            return
        if msg.type == Marker.CUBE_LIST:
            self._latest_preblocked = copy.deepcopy(msg)

    def _on_traversable(self, msg: Marker) -> None:
        if self._is_marker_cleared(msg):
            self._latest_traversable = None
            return
        if msg.type == Marker.CUBE_LIST:
            self._latest_traversable = copy.deepcopy(msg)

    def _on_risk_cost(self, msg: PointCloud2) -> None:
        if msg.width * msg.height == 0:
            self._latest_risk_cost = None
            return
        self._latest_risk_cost = copy.deepcopy(msg)

    def _publish_save_progress(self, message: str) -> None:
        msg = String()
        msg.data = message
        self._save_progress_pub.publish(msg)
        self.get_logger().info(f"Save progress: {message}")

    def _layers_status_message(self) -> str:
        missing = []
        if self._latest_octomap is None:
            missing.append("octomap")
        if self._latest_preblocked is None:
            missing.append("preblocked")
        if self._latest_traversable is None:
            missing.append("traversable")
        if self._latest_risk_cost is None:
            missing.append("risk_cost")
        return ", ".join(missing) if missing else "ready"

    def _wait_for_layers(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._layers_ready():
                return True
            time.sleep(0.2)
        return self._layers_ready()

    def _marker_points_to_numpy(self, marker: Marker) -> np.ndarray:
        if not marker.points:
            return np.empty((0, 3), dtype=np.float32)
        return np.fromiter(
            (coord for p in marker.points for coord in (p.x, p.y, p.z)),
            dtype=np.float32,
            count=len(marker.points) * 3,
        ).reshape(-1, 3)

    def _risk_cost_to_numpy(self, cloud: PointCloud2) -> np.ndarray:
        records = list(
            point_cloud2.read_points(
                cloud,
                field_names=("x", "y", "z", "intensity"),
                skip_nans=True,
            )
        )
        if not records:
            return np.empty((0, 4), dtype=np.float32)
        structured = np.array(records)
        return np.column_stack(
            (
                structured["x"],
                structured["y"],
                structured["z"],
                structured["intensity"],
            )
        ).astype(np.float32, copy=False)

    def _risk_bounds_from_cloud(
        self, cloud: PointCloud2
    ) -> tuple[np.ndarray, np.ndarray] | None:
        min_xyz = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
        max_xyz = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
        found = False
        for x, y, z, _intensity in point_cloud2.read_points(
            cloud,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True,
        ):
            found = True
            min_xyz[0] = min(min_xyz[0], float(x))
            min_xyz[1] = min(min_xyz[1], float(y))
            min_xyz[2] = min(min_xyz[2], float(z))
            max_xyz[0] = max(max_xyz[0], float(x))
            max_xyz[1] = max(max_xyz[1], float(y))
            max_xyz[2] = max(max_xyz[2], float(z))
        if not found:
            return None
        return min_xyz, max_xyz

    def _make_marker_from_points(
        self,
        frame_id: str,
        ns: str,
        scale: np.ndarray,
        points: np.ndarray,
        color: tuple[float, float, float, float],
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = float(scale[0])
        marker.scale.y = float(scale[1])
        marker.scale.z = float(scale[2])
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        for xyz in points:
            point = Point()
            point.x = float(xyz[0])
            point.y = float(xyz[1])
            point.z = float(xyz[2])
            marker.points.append(point)
        return marker

    def _compute_bounds_from_layers(self) -> tuple[list[float], list[float]]:
        chunks: list[np.ndarray] = []
        if self._latest_occupied is not None:
            chunks.append(self._marker_points_to_numpy(self._latest_occupied))
        if self._latest_preblocked is not None:
            chunks.append(self._marker_points_to_numpy(self._latest_preblocked))
        if self._latest_traversable is not None:
            chunks.append(self._marker_points_to_numpy(self._latest_traversable))
        if self._latest_risk_cost is not None:
            risk_bounds = self._risk_bounds_from_cloud(self._latest_risk_cost)
            if risk_bounds is not None:
                min_xyz, max_xyz = risk_bounds
                chunks.append(np.vstack((min_xyz, max_xyz)))
        if not chunks:
            return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]

        min_xyz = np.array([np.inf, np.inf, np.inf], dtype=np.float64)
        max_xyz = np.array([-np.inf, -np.inf, -np.inf], dtype=np.float64)
        for chunk in chunks:
            min_xyz = np.minimum(min_xyz, chunk.min(axis=0))
            max_xyz = np.maximum(max_xyz, chunk.max(axis=0))
        resolution = float(self._latest_octomap.resolution) if self._latest_octomap else 0.2
        half = resolution * 0.5
        return (min_xyz - half).tolist(), (max_xyz + half).tolist()

    def _build_meta_from_cache(self) -> tuple[bool, str, Optional[SimpleNamespace]]:
        if self._latest_octomap is None:
            return False, "octomap message not received yet", None

        min_vals, max_vals = self._compute_bounds_from_layers()
        meta = SimpleNamespace(
            map_id=str(self.get_parameter("planner_map_id").value),
            frame_id=self._latest_octomap.header.frame_id,
            resolution=float(self._latest_octomap.resolution),
            min_bound=Point(x=min_vals[0], y=min_vals[1], z=min_vals[2]),
            max_bound=Point(x=max_vals[0], y=max_vals[1], z=max_vals[2]),
            robot_radius=float(self.get_parameter("planner_robot_radius").value),
            snap_search_radius_cells=int(
                self.get_parameter("planner_snap_search_radius_cells").value
            ),
            require_ground_support=bool(
                self.get_parameter("planner_require_ground_support").value
            ),
            strict_direct_ground_support=bool(
                self.get_parameter("planner_strict_direct_ground_support").value
            ),
            ground_support_xy_radius_cells=int(
                self.get_parameter("planner_ground_support_xy_radius_cells").value
            ),
            ground_support_depth_cells=int(
                self.get_parameter("planner_ground_support_depth_cells").value
            ),
            enable_preblocked_costmap=bool(
                self.get_parameter("planner_enable_preblocked_costmap").value
            ),
            preblocked_costmap_radius_cells=int(
                self.get_parameter("planner_preblocked_costmap_radius_cells").value
            ),
            preblocked_costmap_weight=float(
                self.get_parameter("planner_preblocked_costmap_weight").value
            ),
            source_world_file=str(self.get_parameter("planner_source_world_file").value),
        )
        return True, "ok (local cache)", meta

    def _resolve_save_meta(
        self, skip_export: bool
    ) -> tuple[bool, str, Optional[SimpleNamespace]]:
        if skip_export:
            self._publish_save_progress("规划层已缓存，使用本地元数据。")
            return self._build_meta_from_cache()

        meta_ok, meta_msg, meta_response = self._call_get_meta()
        if meta_ok and meta_response is not None:
            return True, meta_msg, meta_response

        self._publish_save_progress(
            f"规划器 meta 服务不可用（{meta_msg}），改用本地缓存..."
        )
        return self._build_meta_from_cache()

    def _wait_for_future(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            if time.monotonic() > deadline:
                return None
            time.sleep(0.05)
        if not future.done():
            return None
        return future.result()

    def _layers_ready(self) -> bool:
        return (
            self._latest_octomap is not None
            and self._latest_preblocked is not None
            and self._latest_traversable is not None
            and self._latest_risk_cost is not None
        )

    def _call_export_snapshot(
        self, recompute_layers: bool
    ) -> tuple[bool, str, Optional[ExportNavigationSnapshot.Response]]:
        if not self.export_client.wait_for_service(timeout_sec=1.0):
            return False, "planner export service unavailable", None
        request = ExportNavigationSnapshot.Request()
        request.recompute_layers = recompute_layers
        future = self.export_client.call_async(request)
        timeout_sec = float(self.get_parameter("save_export_timeout_sec").value)
        result = self._wait_for_future(future, timeout_sec)
        if result is None:
            return False, f"planner export service timed out ({timeout_sec:.0f}s)", None
        return result.success, result.message, result

    def _call_get_meta(self) -> tuple[bool, str, Optional[GetNavigationMapMeta.Response]]:
        if not self.meta_client.wait_for_service(timeout_sec=2.0):
            return False, "planner meta service unavailable", None
        future = self.meta_client.call_async(GetNavigationMapMeta.Request())
        timeout_sec = float(self.get_parameter("save_meta_timeout_sec").value)
        result = self._wait_for_future(future, timeout_sec)
        if result is None:
            return False, f"planner meta service timed out ({timeout_sec:.0f}s)", None
        return result.success, result.message, result

    def _handle_save_package(
        self,
        request: SaveNavigationMapPackage.Request,
        response: SaveNavigationMapPackage.Response,
    ) -> SaveNavigationMapPackage.Response:
        started = time.monotonic()
        package_dir = Path(request.package_path).expanduser()
        self._publish_save_progress(f"开始保存地图：{package_dir}")

        skip_export = (
            bool(self.get_parameter("save_skip_export_if_ready").value)
            and self._layers_ready()
        )
        if skip_export:
            self._publish_save_progress("规划层已缓存，跳过 export_snapshot。")
            snapshot_stamp = self._latest_octomap.header.stamp
        else:
            missing = self._layers_status_message()
            self._publish_save_progress(
                f"规划层未就绪（缺少: {missing}），请求规划器刷新..."
            )
            recompute_layers = bool(self.get_parameter("save_recompute_layers").value)
            export_ok, export_msg, export_result = self._call_export_snapshot(recompute_layers)
            if not export_ok or export_result is None:
                response.success = False
                response.message = export_msg
                self._publish_save_progress(f"保存失败：{export_msg}")
                return response
            sync_timeout = float(self.get_parameter("save_layer_sync_timeout_sec").value)
            self._publish_save_progress(
                f"等待规划层同步（最多 {sync_timeout:.0f}s）..."
            )
            if not self._wait_for_layers(sync_timeout):
                response.success = False
                response.message = (
                    "规划层同步超时。请等待终端出现 "
                    "'Preprocess mask rebuilt' 后再保存。"
                )
                self._publish_save_progress(response.message)
                return response
            snapshot_stamp = export_result.snapshot_stamp

        self._publish_save_progress("读取规划元数据...")
        meta_ok, meta_msg, meta = self._resolve_save_meta(skip_export)
        if not meta_ok or meta is None:
            response.success = False
            response.message = meta_msg
            self._publish_save_progress(f"保存失败：{meta_msg}")
            return response

        if self._latest_octomap is None:
            response.success = False
            response.message = "octomap message not received yet"
            self._publish_save_progress(f"保存失败：{response.message}")
            return response
        if self._latest_preblocked is None:
            response.success = False
            response.message = "preblocked marker not received yet"
            self._publish_save_progress(f"保存失败：{response.message}")
            return response
        if self._latest_traversable is None:
            response.success = False
            response.message = "traversable marker not received yet"
            self._publish_save_progress(f"保存失败：{response.message}")
            return response
        if self._latest_risk_cost is None:
            response.success = False
            response.message = "risk cost cloud not received yet"
            self._publish_save_progress(f"保存失败：{response.message}")
            return response

        if package_dir.exists():
            if not request.overwrite:
                response.success = False
                response.message = f"package path already exists: {package_dir}"
                self._publish_save_progress(f"保存失败：{response.message}")
                return response
        package_dir.mkdir(parents=True, exist_ok=True)

        octomap_file = package_dir / "octomap_msg.npz"
        layers_file = package_dir / "layers.npz"
        meta_file = package_dir / "meta.yaml"

        self._publish_save_progress(
            f"写入 OctoMap（{len(self._latest_octomap.data)} bytes）..."
        )
        np.savez_compressed(
            octomap_file,
            binary=np.array([self._latest_octomap.binary], dtype=np.bool_),
            octomap_id=np.array([self._latest_octomap.id]),
            resolution=np.array([self._latest_octomap.resolution], dtype=np.float64),
            frame_id=np.array([self._latest_octomap.header.frame_id]),
            data=np.array(self._latest_octomap.data, dtype=np.int8),
        )

        self._publish_save_progress("转换并写入规划图层...")
        preblocked_points = self._marker_points_to_numpy(self._latest_preblocked)
        traversable_points = self._marker_points_to_numpy(self._latest_traversable)
        risk_points = self._risk_cost_to_numpy(self._latest_risk_cost)
        self._publish_save_progress(
            f"压缩图层（preblocked={preblocked_points.shape[0]}, "
            f"traversable={traversable_points.shape[0]}, "
            f"risk={risk_points.shape[0]}）..."
        )
        np.savez_compressed(
            layers_file,
            preblocked_points=preblocked_points,
            preblocked_scale=np.array(
                [
                    self._latest_preblocked.scale.x,
                    self._latest_preblocked.scale.y,
                    self._latest_preblocked.scale.z,
                ],
                dtype=np.float64,
            ),
            preblocked_frame_id=np.array([self._latest_preblocked.header.frame_id]),
            traversable_points=traversable_points,
            traversable_scale=np.array(
                [
                    self._latest_traversable.scale.x,
                    self._latest_traversable.scale.y,
                    self._latest_traversable.scale.z,
                ],
                dtype=np.float64,
            ),
            traversable_frame_id=np.array([self._latest_traversable.header.frame_id]),
            risk_points=risk_points[:, :3] if risk_points.size else np.empty((0, 3), dtype=np.float32),
            risk_intensity=risk_points[:, 3] if risk_points.size else np.empty((0,), dtype=np.float32),
            risk_frame_id=np.array([self._latest_risk_cost.header.frame_id]),
        )

        self._publish_save_progress("写入 meta.yaml...")
        meta_yaml = {
            "map_id": meta.map_id,
            "frame_id": meta.frame_id,
            "resolution": meta.resolution,
            "octomap_file": octomap_file.name,
            "layers_file": layers_file.name,
            "source_world_file": meta.source_world_file,
            "snapshot_stamp": {
                "sec": int(snapshot_stamp.sec),
                "nanosec": int(snapshot_stamp.nanosec),
            },
            "bounds": {
                "min": [meta.min_bound.x, meta.min_bound.y, meta.min_bound.z],
                "max": [meta.max_bound.x, meta.max_bound.y, meta.max_bound.z],
            },
            "planner": {
                "robot_radius": meta.robot_radius,
                "snap_search_radius_cells": meta.snap_search_radius_cells,
                "require_ground_support": meta.require_ground_support,
                "strict_direct_ground_support": meta.strict_direct_ground_support,
                "ground_support_xy_radius_cells": meta.ground_support_xy_radius_cells,
                "ground_support_depth_cells": meta.ground_support_depth_cells,
                "enable_preblocked_costmap": meta.enable_preblocked_costmap,
                "preblocked_costmap_radius_cells": meta.preblocked_costmap_radius_cells,
                "preblocked_costmap_weight": meta.preblocked_costmap_weight,
            },
            "layers": {
                "preblocked_count": int(preblocked_points.shape[0]),
                "traversable_count": int(traversable_points.shape[0]),
                "risk_cost_count": int(risk_points.shape[0]),
            },
        }

        with meta_file.open("w", encoding="utf-8") as f:
            yaml.safe_dump(meta_yaml, f, sort_keys=False, allow_unicode=True)

        response.success = True
        response.message = "map package saved"
        response.manifest_path = str(meta_file)
        elapsed = time.monotonic() - started
        done_msg = (
            f"保存完成：{package_dir}（耗时 {elapsed:.1f}s，"
            f"preblocked={int(preblocked_points.shape[0])}, "
            f"traversable={int(traversable_points.shape[0])}, "
            f"risk={int(risk_points.shape[0])}）"
        )
        self._publish_save_progress(done_msg)
        self.get_logger().info(
            f"Saved map package to {package_dir} in {elapsed:.1f}s "
            f"(preblocked={int(preblocked_points.shape[0])} "
            f"traversable={int(traversable_points.shape[0])} "
            f"risk={int(risk_points.shape[0])})"
        )
        return response

    def _handle_load_package(
        self,
        request: LoadNavigationMapPackage.Request,
        response: LoadNavigationMapPackage.Response,
    ) -> LoadNavigationMapPackage.Response:
        success, message, map_id = self._load_package(request.package_path)
        response.success = success
        response.message = message
        response.map_id = map_id
        return response

    def _autoload_package_once(self) -> None:
        self._autoload_timer.cancel()
        package_path = str(self.get_parameter("autoload_package_path").value).strip()
        if not package_path:
            return

        success, message, map_id = self._load_package(package_path)
        if success:
            self.get_logger().info(
                f"autoloaded map package: {package_path} map_id={map_id}"
            )
        else:
            self.get_logger().error(
                f"failed to autoload map package {package_path}: {message}"
            )

    def _load_package(self, package_path: str) -> tuple[bool, str, str]:
        package_dir = Path(package_path).expanduser()
        meta_file = package_dir / "meta.yaml"
        if not meta_file.exists():
            return False, f"meta file not found: {meta_file}", ""

        with meta_file.open("r", encoding="utf-8") as f:
            meta = yaml.safe_load(f)

        octomap_npz = np.load(package_dir / meta["octomap_file"], allow_pickle=False)
        layers_npz = np.load(package_dir / meta["layers_file"], allow_pickle=False)
        frame_id = str(self.get_parameter("publish_frame_id").value)
        stamp = self.get_clock().now().to_msg()

        octomap_msg = Octomap()
        octomap_msg.header.frame_id = frame_id
        octomap_msg.header.stamp = stamp
        octomap_msg.binary = bool(octomap_npz["binary"][0])
        octomap_msg.id = str(octomap_npz["octomap_id"][0])
        octomap_msg.resolution = float(octomap_npz["resolution"][0])
        octomap_msg.data = octomap_npz["data"].astype(np.int8).tolist()

        occupied_msg = None
        if "occupied_points" in layers_npz:
            occupied_msg = self._make_marker_from_points(
                frame_id,
                "occupied_voxels",
                layers_npz["occupied_scale"],
                layers_npz["occupied_points"],
                (0.95, 0.45, 0.15, 0.95),
            )
            occupied_msg.header.stamp = stamp
        preblocked_msg = self._make_marker_from_points(
            frame_id,
            "preblocked_cells",
            layers_npz["preblocked_scale"],
            layers_npz["preblocked_points"],
            (0.15, 0.35, 1.0, 0.95),
        )
        preblocked_msg.header.stamp = stamp
        traversable_msg = self._make_marker_from_points(
            frame_id,
            "traversable_cells",
            layers_npz["traversable_scale"],
            layers_npz["traversable_points"],
            (0.20, 0.95, 0.55, 0.55),
        )
        traversable_msg.header.stamp = stamp
        risk_header = PointCloud2()
        risk_header.header.frame_id = frame_id
        risk_header.header.stamp = stamp
        risk_points = layers_npz["risk_points"]
        risk_intensity = layers_npz["risk_intensity"]
        risk_msg = point_cloud2.create_cloud(
            risk_header.header,
            [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
                PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            ],
            [
                (float(p[0]), float(p[1]), float(p[2]), float(i))
                for p, i in zip(risk_points, risk_intensity)
            ],
        )

        self._latest_octomap = copy.deepcopy(octomap_msg)
        self._latest_occupied = copy.deepcopy(occupied_msg) if occupied_msg is not None else None
        self._latest_preblocked = copy.deepcopy(preblocked_msg)
        self._latest_traversable = copy.deepcopy(traversable_msg)
        self._latest_risk_cost = copy.deepcopy(risk_msg)
        octomap_qos = self._make_layer_qos(self._octomap_qos_transient_local)
        layer_qos = self._make_layer_qos(self._layer_qos_transient_local)
        self._ensure_layer_publishers(octomap_qos, layer_qos)
        if bool(self.get_parameter("clear_navigation_layers_on_load").value):
            self._clear_navigation_layers_in_rviz(frame_id)
        self.octomap_pub.publish(octomap_msg)
        if occupied_msg is not None and self.occupied_pub is not None:
            self.occupied_pub.publish(occupied_msg)
        self.preblocked_pub.publish(preblocked_msg)
        self.traversable_pub.publish(traversable_msg)
        self.risk_cost_pub.publish(risk_msg)

        self.get_logger().info(
            f"map package loaded: {package_dir} frame_id={frame_id} "
            f"preblocked_pts={len(preblocked_msg.points)} "
            f"traversable_pts={len(traversable_msg.points)} "
            f"risk_pts={len(risk_points)}"
        )
        return True, "map package loaded", str(meta.get("map_id", ""))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapPackageManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
