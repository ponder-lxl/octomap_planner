#!/usr/bin/env python3

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from map_3d_msgs.srv import SaveNavigationMapPackage
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from vtkmodules.qt.QVTKRenderWindowInteractor import QVTKRenderWindowInteractor
import vtk
from vtk.util import numpy_support


class WorldSelectorRosNode(Node):
    DEFAULT_SAVE_TIMEOUT_SEC = 600.0

    def __init__(self) -> None:
        super().__init__(f"world_selector_gui_node_{time.time_ns()}")
        self.declare_parameter("initial_world_file", "")
        self.declare_parameter("world_file_cmd_topic", "/world_file_cmd")
        self.declare_parameter("occupied_marker_topic", "/octomap_occupied_markers")
        self.declare_parameter("preblocked_topic", "/preblocked_cells_markers")
        self.declare_parameter("traversable_topic", "/traversable_cells_markers")
        self.declare_parameter("risk_cost_topic", "/risk_cost_cells")
        self.declare_parameter("start_topic", "/start_point")
        self.declare_parameter("goal_topic", "/goal_point")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("path_topic", "/planned_path")
        self.declare_parameter("save_progress_topic", "/map_package_manager/save_progress")

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.world_file_pub = self.create_publisher(
            String, self.get_parameter("world_file_cmd_topic").value, qos
        )
        self.start_pub = self.create_publisher(
            PointStamped, self.get_parameter("start_topic").value, qos
        )
        self.goal_pub = self.create_publisher(
            PointStamped, self.get_parameter("goal_topic").value, qos
        )
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, self.get_parameter("goal_pose_topic").value, qos
        )
        self.occupied_sub = self.create_subscription(
            Marker, self.get_parameter("occupied_marker_topic").value, self._on_occupied, qos
        )
        self.preblocked_sub = self.create_subscription(
            Marker, self.get_parameter("preblocked_topic").value, self._on_preblocked, qos
        )
        self.traversable_sub = self.create_subscription(
            Marker, self.get_parameter("traversable_topic").value, self._on_traversable, qos
        )
        self.risk_sub = self.create_subscription(
            PointCloud2, self.get_parameter("risk_cost_topic").value, self._on_risk, qos
        )
        self.path_sub = self.create_subscription(
            PathMsg, self.get_parameter("path_topic").value, self._on_path, qos
        )
        self.save_progress_sub = self.create_subscription(
            String,
            self.get_parameter("save_progress_topic").value,
            self._on_save_progress,
            QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            ),
        )

        self._latest_occupied: tuple[np.ndarray, np.ndarray] | None = None
        self._latest_preblocked: tuple[np.ndarray, np.ndarray] | None = None
        self._latest_traversable: tuple[np.ndarray, np.ndarray] | None = None
        self._latest_risk: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
        self._latest_path_points: list[tuple[float, float, float]] = []
        self._layer_dirty = False
        self._path_dirty = False
        self._save_progress_dirty = False
        self._latest_save_progress = ""
        self._save_client = self.create_client(
            SaveNavigationMapPackage, "/map_package_manager/save_package"
        )
        self._save_future = None
        self._save_started_at: float | None = None
        self._save_timeout_sec = self.DEFAULT_SAVE_TIMEOUT_SEC

    def start_save_package(
        self,
        package_path: str,
        overwrite: bool,
        timeout_sec: float = DEFAULT_SAVE_TIMEOUT_SEC,
    ) -> tuple[bool, str]:
        if self._save_future is not None and not self._save_future.done():
            return False, "已有保存任务正在进行。"
        if not self._save_client.wait_for_service(timeout_sec=2.0):
            return False, "保存服务 /map_package_manager/save_package 不可用。"
        request = SaveNavigationMapPackage.Request()
        request.package_path = package_path
        request.overwrite = overwrite
        self._save_timeout_sec = timeout_sec
        self._save_started_at = time.monotonic()
        self._save_future = self._save_client.call_async(request)
        return True, ""

    def poll_save_package(self) -> tuple[bool, str] | None:
        if self._save_future is None:
            return None
        if not self._save_future.done():
            if (
                self._save_started_at is not None
                and time.monotonic() - self._save_started_at > self._save_timeout_sec
            ):
                self._save_future = None
                self._save_started_at = None
                return (
                    False,
                    f"保存地图超时（>{self._save_timeout_sec:.0f}s）。"
                    "大型地图请耐心等待或缩小 xy_window_size_m。",
                )
            return None
        result = self._save_future.result()
        self._save_future = None
        self._save_started_at = None
        if result is None:
            return False, "保存地图失败，服务没有返回结果。"
        return bool(result.success), str(result.message)

    def consume_save_progress(self) -> str | None:
        if not self._save_progress_dirty:
            return None
        self._save_progress_dirty = False
        return self._latest_save_progress

    def _on_save_progress(self, msg: String) -> None:
        self._latest_save_progress = str(msg.data)
        self._save_progress_dirty = True

    def initial_world_file(self) -> str:
        return str(self.get_parameter("initial_world_file").value)

    def publish_world_file(self, world_file: str) -> None:
        msg = String()
        msg.data = world_file
        self.world_file_pub.publish(msg)

    def publish_point(self, topic: str, frame_id: str, xyz: tuple[float, float, float]) -> None:
        msg = PointStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(xyz[0])
        msg.point.y = float(xyz[1])
        msg.point.z = float(xyz[2])
        if topic == "start":
            self.start_pub.publish(msg)
        else:
            self.goal_pub.publish(msg)

    def publish_goal_pose(
        self, frame_id: str, xyz: tuple[float, float, float], yaw: float
    ) -> None:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        half_yaw = float(yaw) * 0.5
        msg.pose.orientation.z = float(np.sin(half_yaw))
        msg.pose.orientation.w = float(np.cos(half_yaw))
        self.goal_pose_pub.publish(msg)

    def _on_occupied(self, msg: Marker) -> None:
        self._store_marker("occupied", msg)

    def _on_preblocked(self, msg: Marker) -> None:
        self._store_marker("preblocked", msg)

    def _on_traversable(self, msg: Marker) -> None:
        self._store_marker("traversable", msg)

    def _on_risk(self, msg: PointCloud2) -> None:
        records = list(
            point_cloud2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
            )
        )
        if not records:
            xyz = np.empty((0, 3), dtype=np.float32)
            intensity = np.empty((0,), dtype=np.float32)
        else:
            points = np.array(
                [[row[0], row[1], row[2], row[3]] for row in records],
                dtype=np.float32,
            )
            xyz = points[:, :3]
            intensity = points[:, 3]
        scale = self._infer_voxel_scale()
        self._latest_risk = (xyz, scale, intensity)
        self._layer_dirty = True

    def _on_path(self, msg: PathMsg) -> None:
        self._latest_path_points = [
            (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z)
            for pose in msg.poses
        ]
        self._path_dirty = True

    def _store_marker(self, layer_name: str, msg: Marker) -> None:
        if msg.type != Marker.CUBE_LIST:
            return
        points = np.array([[p.x, p.y, p.z] for p in msg.points], dtype=np.float32)
        scale = np.array([msg.scale.x, msg.scale.y, msg.scale.z], dtype=np.float32)
        setattr(self, f"_latest_{layer_name}", (points, scale))
        self._layer_dirty = True

    def consume_layers(self):
        if not self._layer_dirty:
            return None
        self._layer_dirty = False
        return {
            "occupied": self._latest_occupied,
            "preblocked": self._latest_preblocked,
            "traversable": self._latest_traversable,
            "risk": self._latest_risk,
        }

    def consume_path(self) -> list[tuple[float, float, float]] | None:
        if not self._path_dirty:
            return None
        self._path_dirty = False
        return list(self._latest_path_points)

    def _infer_voxel_scale(self) -> np.ndarray:
        for payload in (self._latest_occupied, self._latest_preblocked, self._latest_traversable):
            if payload is not None:
                return np.asarray(payload[1], dtype=np.float32)
        return np.array([0.2, 0.2, 0.2], dtype=np.float32)


class WorldSelectorWindow(QWidget):
    _LAYER_STYLE = {
        "occupied": ((0.95, 0.45, 0.15), 1.0, "占据"),
        "preblocked": ((0.15, 0.35, 1.0), 1.0, "禁行"),
        "traversable": ((0.20, 0.95, 0.55), 0.30, "可通行"),
        "risk": ((0.15, 0.35, 1.0), 0.55, "风险代价"),
    }
    _GENERATION_LAYER_ORDER = ("occupied", "preblocked", "traversable", "risk")
    _GENERATION_LAYER_LABELS = {
        "occupied": "占据",
        "preblocked": "禁行",
        "traversable": "可通行",
        "risk": "风险代价",
    }

    def __init__(self) -> None:
        super().__init__()
        self._ros_node = WorldSelectorRosNode()
        self._default_root = Path("/home/robot/maps")
        self._saving = False
        self._pending_save_path: Path | None = None
        self._renderer = vtk.vtkRenderer()
        self._layer_actors: dict[str, tuple[vtk.vtkActor, vtk.vtkActor | None]] = {}
        self._layer_data: dict[str, tuple[np.ndarray, np.ndarray, tuple[float, float, float], float] | tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
        self._camera_initialized = False
        self._frame_id = "map"
        self._pick_mode: str | None = None
        self._start_actor: vtk.vtkActor | None = None
        self._goal_actor: vtk.vtkActor | None = None
        self._goal_arrow_actor: vtk.vtkActor | None = None
        self._goal_pending_position: tuple[float, float, float] | None = None
        self._goal_yaw: float = 0.0
        self._path_actor: vtk.vtkActor | None = None
        self._generation_ready = {name: False for name in self._GENERATION_LAYER_ORDER}
        self._generation_counts = {name: 0 for name in self._GENERATION_LAYER_ORDER}
        self._generation_complete = False
        self._init_ui()
        self._spin_timer = QTimer(self)
        self._spin_timer.timeout.connect(self._spin_ros_once)
        self._spin_timer.start(100)

    def _init_ui(self) -> None:
        self.setWindowTitle("World Selector - OctoMap 3D")
        self.resize(1180, 760)

        root = QVBoxLayout()
        top_row = QHBoxLayout()

        left_panel = QVBoxLayout()

        world_group = QGroupBox("World 文件")
        world_form = QFormLayout()
        world_row = QHBoxLayout()
        self.path_edit = QLineEdit(self._ros_node.initial_world_file())
        self.path_edit.setPlaceholderText("选择 .world 或 .sdf 文件")
        browse_btn = QPushButton("浏览")
        browse_btn.clicked.connect(self._browse_world)
        world_row.addWidget(self.path_edit, 1)
        world_row.addWidget(browse_btn)
        world_form.addRow("文件路径", world_row)

        load_btn = QPushButton("加载World")
        load_btn.clicked.connect(self._load_world)
        world_form.addRow("", load_btn)
        world_group.setLayout(world_form)

        save_group = QGroupBox("OctoMap地图保存")
        save_form = QFormLayout()
        save_root_row = QHBoxLayout()
        self.save_root_edit = QLineEdit(str(self._default_root))
        save_root_btn = QPushButton("选择目录")
        save_root_btn.clicked.connect(self._choose_save_root)
        save_root_row.addWidget(self.save_root_edit, 1)
        save_root_row.addWidget(save_root_btn)
        save_form.addRow("根目录", save_root_row)

        self.save_name_edit = QLineEdit()
        self.save_name_edit.setPlaceholderText("请输入地图名，例如 lv2_map")
        world_path = self.path_edit.text().strip()
        if world_path:
            self.save_name_edit.setText(Path(world_path).stem)
        save_form.addRow("地图名", self.save_name_edit)

        self.overwrite_checkbox = QCheckBox("允许覆盖")
        self.overwrite_checkbox.setChecked(True)
        save_form.addRow("", self.overwrite_checkbox)

        save_btn = QPushButton("保存OctoMap地图")
        save_btn.clicked.connect(self._start_save)
        save_form.addRow("", save_btn)
        save_group.setLayout(save_form)

        navigation_group = QGroupBox("导航设置")
        navigation_row = QHBoxLayout()
        self.start_btn = QPushButton("起始点")
        self.start_btn.clicked.connect(lambda: self._set_pick_mode("start"))
        self.goal_btn = QPushButton("目标点")
        self.goal_btn.clicked.connect(lambda: self._set_pick_mode("goal"))
        navigation_row.addWidget(self.start_btn)
        navigation_row.addWidget(self.goal_btn)
        navigation_row.addStretch(1)
        navigation_group.setLayout(navigation_row)

        display_group = QGroupBox("显示图层")
        display_row = QHBoxLayout()
        self.occupied_checkbox = QCheckBox("占据")
        self.occupied_checkbox.setChecked(True)
        self.occupied_checkbox.toggled.connect(self._refresh_layers)
        self.preblocked_checkbox = QCheckBox("禁行")
        self.preblocked_checkbox.setChecked(True)
        self.preblocked_checkbox.toggled.connect(self._refresh_layers)
        self.traversable_checkbox = QCheckBox("可通行")
        self.traversable_checkbox.setChecked(False)
        self.traversable_checkbox.toggled.connect(self._refresh_layers)
        self.risk_checkbox = QCheckBox("风险代价")
        self.risk_checkbox.setChecked(True)
        self.risk_checkbox.toggled.connect(self._refresh_layers)
        display_row.addWidget(self.occupied_checkbox)
        display_row.addWidget(self.preblocked_checkbox)
        display_row.addWidget(self.traversable_checkbox)
        display_row.addWidget(self.risk_checkbox)
        display_row.addStretch(1)
        display_group.setLayout(display_row)

        self.status_label = QLabel("等待 world 加载。")
        self.status_label.setWordWrap(True)
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)

        left_panel.addWidget(world_group)
        left_panel.addWidget(save_group)
        left_panel.addWidget(navigation_group)
        left_panel.addWidget(display_group)
        left_panel.addWidget(self.status_label)
        left_panel.addStretch(1)
        left_panel_widget = QWidget()
        left_panel_widget.setLayout(left_panel)
        left_panel_widget.setFixedWidth(360)

        self.vtk_widget = QVTKRenderWindowInteractor(self)
        self.vtk_widget.GetRenderWindow().AddRenderer(self._renderer)
        self._renderer.SetBackground(0.04, 0.07, 0.09)
        self._renderer.GradientBackgroundOn()
        self._renderer.SetBackground2(0.12, 0.16, 0.19)
        axes = vtk.vtkAxesActor()
        axes.SetTotalLength(1.5, 1.5, 1.5)
        axes.SetXAxisLabelText("")
        axes.SetYAxisLabelText("")
        axes.SetZAxisLabelText("")
        self._renderer.AddActor(axes)
        self._renderer.AddActor(self._make_ground_grid(24.0, 1.0))
        interactor = self.vtk_widget.GetRenderWindow().GetInteractor()
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleTrackballCamera())
        interactor.Initialize()
        interactor.AddObserver("LeftButtonPressEvent", self._on_left_button_press, 1.0)
        interactor.AddObserver("MouseMoveEvent", self._on_mouse_move, 1.0)

        top_row.addWidget(left_panel_widget, 0)
        top_row.addWidget(self.vtk_widget, 1)
        root.addLayout(top_row)
        self.setLayout(root)

        if self.path_edit.text().strip():
            self.status_label.setText(
                f"当前 launch 初始 world：{Path(self.path_edit.text().strip()).name}。"
                " 后端会自动生成 OctoMap。"
            )

    def closeEvent(self, event) -> None:
        self._ros_node.destroy_node()
        super().closeEvent(event)

    def _browse_world(self) -> None:
        selected, _ = QFileDialog.getOpenFileName(
            self,
            "选择 Gazebo World/SDF",
            str(Path.home()),
            "World Files (*.world *.sdf);;All Files (*)",
        )
        if selected:
            self.path_edit.setText(selected)
            if not self.save_name_edit.text().strip():
                self.save_name_edit.setText(Path(selected).stem)

    def _choose_save_root(self) -> None:
        selected = QFileDialog.getExistingDirectory(
            self,
            "选择地图保存根目录",
            self.save_root_edit.text().strip() or str(self._default_root),
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )
        if selected:
            self.save_root_edit.setText(selected)

    def _load_world(self) -> None:
        world_file = self.path_edit.text().strip()
        if not world_file:
            QMessageBox.warning(self, "World 文件", "请先选择 world 文件。")
            return
        path = Path(world_file)
        if not path.is_file():
            QMessageBox.warning(self, "World 文件", f"文件不存在：{path}")
            return

        self._layer_data.clear()
        self._camera_initialized = False
        self._reset_generation_tracking()
        self._refresh_layers()
        self._ros_node.publish_world_file(str(path))
        self.status_label.setText(
            f"已发送 world 文件：{path.name}。后端正在转换为 3D OctoMap，请稍候。"
        )

    def _reset_generation_tracking(self) -> None:
        self._generation_ready = {name: False for name in self._GENERATION_LAYER_ORDER}
        self._generation_counts = {name: 0 for name in self._GENERATION_LAYER_ORDER}
        self._generation_complete = False

    def _mark_layer_generated(self, layer_name: str, count: int) -> None:
        if layer_name not in self._generation_ready:
            return
        self._generation_ready[layer_name] = True
        self._generation_counts[layer_name] = int(count)
        self._update_generation_status()

    def _update_generation_status(self) -> None:
        if self._saving or self._pick_mode is not None:
            return

        ready_count = sum(1 for ready in self._generation_ready.values() if ready)
        total_count = len(self._GENERATION_LAYER_ORDER)
        if ready_count == total_count:
            if not self._generation_complete:
                self._generation_complete = True
                counts = self._generation_counts
                message = (
                    "全部图层生成完成："
                    f"占据 {counts['occupied']}，禁行 {counts['preblocked']}，"
                    f"可通行 {counts['traversable']}，风险 {counts['risk']}。"
                )
                self.status_label.setText(message)
                self._ros_node.get_logger().info(message)
            return

        parts: list[str] = []
        for layer_name in self._GENERATION_LAYER_ORDER:
            label = self._GENERATION_LAYER_LABELS[layer_name]
            if self._generation_ready[layer_name]:
                parts.append(f"{label} ✓({self._generation_counts[layer_name]})")
            else:
                parts.append(f"{label} …")
        self.status_label.setText(
            f"图层生成进度 {ready_count}/{total_count}：" + "，".join(parts)
        )

    def _build_package_path(self) -> Path | None:
        root_dir = self.save_root_edit.text().strip()
        map_name = self.save_name_edit.text().strip()
        if not root_dir:
            QMessageBox.warning(self, "保存地图", "请先选择地图根目录。")
            return None
        if not map_name:
            QMessageBox.warning(self, "保存地图", "请输入地图名。")
            return None
        return Path(root_dir).expanduser() / map_name

    def _set_save_busy(self, busy: bool) -> None:
        self.save_root_edit.setEnabled(not busy)
        self.save_name_edit.setEnabled(not busy)
        self.overwrite_checkbox.setEnabled(not busy)

    def _start_save(self) -> None:
        package_path = self._build_package_path()
        if package_path is None:
            return
        ok, message = self._ros_node.start_save_package(
            str(package_path),
            self.overwrite_checkbox.isChecked(),
        )
        if not ok:
            QMessageBox.critical(self, "保存地图", message)
            return
        self._saving = True
        self._pending_save_path = package_path
        self._set_save_busy(True)
        self.status_label.setText(
            f"正在保存地图到 {package_path} ，大型地图可能需要数分钟，请稍候。"
        )

    def _on_save_finished(self, success: bool, message: str) -> None:
        self._saving = False
        self._pending_save_path = None
        self._set_save_busy(False)
        if success:
            self.status_label.setText(message or "地图保存成功。")
            QMessageBox.information(self, "保存地图", "地图保存成功。")
        else:
            self.status_label.setText(message)
            QMessageBox.critical(self, "保存地图", message)

    def _spin_ros_once(self) -> None:
        rclpy.spin_once(self._ros_node, timeout_sec=0.0)
        save_result = self._ros_node.poll_save_package()
        if save_result is not None:
            self._on_save_finished(*save_result)
        save_progress = self._ros_node.consume_save_progress()
        if save_progress is not None and self._saving:
            self.status_label.setText(f"保存进度：{save_progress}")
        layers = self._ros_node.consume_layers()
        if layers is None:
            path_points = self._ros_node.consume_path()
            if path_points is not None:
                self._update_path(path_points)
            return
        for layer_name, payload in layers.items():
            if payload is None:
                continue
            if layer_name == "risk":
                self._layer_data[layer_name] = payload
                self._mark_layer_generated("risk", len(payload[0]))
            else:
                points, scale = payload
                color, opacity, _label = self._LAYER_STYLE[layer_name]
                self._layer_data[layer_name] = (points, scale, color, opacity)
                self._mark_layer_generated(layer_name, len(points))
        self._refresh_layers()
        if not self._saving and not self._generation_complete:
            self._update_generation_status()
        path_points = self._ros_node.consume_path()
        if path_points is not None:
            self._update_path(path_points)

    def _set_pick_mode(self, mode: str) -> None:
        self._pick_mode = mode
        self._goal_pending_position = None
        if mode == "start":
            self.status_label.setText("点击 3D 视图设置起始点。")
        else:
            self.status_label.setText("点击 3D 视图设置目标点位置，再点击一次设置目标朝向。")

    def _on_left_button_press(self, obj, event) -> None:
        if self._pick_mode is None:
            return

        actor_list = [actor for actor, _edge_actor in self._layer_actors.values()]
        if not actor_list:
            return

        click_x, click_y = obj.GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.PickFromListOn()
        for actor in actor_list:
            picker.AddPickList(actor)
        if picker.Pick(click_x, click_y, 0, self._renderer) == 0:
            self.status_label.setText("没有选中栅格。")
            return

        pos = picker.GetPickPosition()
        picked_xyz = self._snap_pick_to_navigation_cell(
            (float(pos[0]), float(pos[1]), float(pos[2]))
        )
        mode = self._pick_mode
        if mode == "start":
            self._pick_mode = None
            self._ros_node.publish_point("start", self._frame_id, picked_xyz)
            self._update_point_actor("start", picked_xyz)
            self.status_label.setText(
                f"起始点已设置：[{picked_xyz[0]:.2f}, {picked_xyz[1]:.2f}, {picked_xyz[2]:.2f}]"
            )
            return

        if mode == "goal":
            self._goal_pending_position = picked_xyz
            self._pick_mode = "goal_heading"
            self._update_goal_visual(picked_xyz, self._goal_yaw)
            self.status_label.setText("目标点位置已设置。移动鼠标预览朝向，再点击一次确认目标姿态。")
            return

        if mode == "goal_heading" and self._goal_pending_position is not None:
            yaw = self._compute_goal_yaw(self._goal_pending_position, picked_xyz)
            self._goal_yaw = yaw
            self._pick_mode = None
            goal_xyz = self._goal_pending_position
            self._goal_pending_position = None
            self._ros_node.publish_point("goal", self._frame_id, goal_xyz)
            self._ros_node.publish_goal_pose(self._frame_id, goal_xyz, yaw)
            self._update_goal_visual(goal_xyz, yaw)
            self.status_label.setText(
                f"目标点已设置：[{goal_xyz[0]:.2f}, {goal_xyz[1]:.2f}, {goal_xyz[2]:.2f}]，"
                f"朝向 {np.degrees(yaw):.1f}°，正在规划路径。"
            )

    def _on_mouse_move(self, obj, event) -> None:
        if self._pick_mode != "goal_heading" or self._goal_pending_position is None:
            return
        actor_list = [actor for actor, _edge_actor in self._layer_actors.values()]
        if not actor_list:
            return
        move_x, move_y = obj.GetEventPosition()
        picker = vtk.vtkPropPicker()
        picker.PickFromListOn()
        for actor in actor_list:
            picker.AddPickList(actor)
        if picker.Pick(move_x, move_y, 0, self._renderer) == 0:
            return
        pos = picker.GetPickPosition()
        cursor_xyz = self._snap_pick_to_navigation_cell(
            (float(pos[0]), float(pos[1]), float(pos[2]))
        )
        yaw = self._compute_goal_yaw(self._goal_pending_position, cursor_xyz)
        self._goal_yaw = yaw
        self._update_goal_visual(self._goal_pending_position, yaw)

    def _compute_goal_yaw(
        self,
        goal_xyz: tuple[float, float, float],
        facing_xyz: tuple[float, float, float],
    ) -> float:
        dx = float(facing_xyz[0] - goal_xyz[0])
        dy = float(facing_xyz[1] - goal_xyz[1])
        if abs(dx) < 1.0e-6 and abs(dy) < 1.0e-6:
            return self._goal_yaw
        return float(np.arctan2(dy, dx))

    def _snap_pick_to_navigation_cell(
        self, xyz: tuple[float, float, float]
    ) -> tuple[float, float, float]:
        layer = self._layer_data.get("traversable")
        if layer is None:
            return xyz
        points = layer[0]
        if points.size == 0:
            return xyz
        diffs = points - np.asarray(xyz, dtype=np.float32)
        nearest_index = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))
        snapped = points[nearest_index]
        return (float(snapped[0]), float(snapped[1]), float(snapped[2]))

    def _make_ground_grid(self, size: float, step: float) -> vtk.vtkActor:
        append = vtk.vtkAppendPolyData()
        half = int(size / step)
        for i in range(-half, half + 1):
            line_x = vtk.vtkLineSource()
            line_x.SetPoint1(-size, i * step, 0.0)
            line_x.SetPoint2(size, i * step, 0.0)
            append.AddInputConnection(line_x.GetOutputPort())
            line_y = vtk.vtkLineSource()
            line_y.SetPoint1(i * step, -size, 0.0)
            line_y.SetPoint2(i * step, size, 0.0)
            append.AddInputConnection(line_y.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(append.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.24, 0.30, 0.34)
        actor.GetProperty().SetLineWidth(1.0)
        actor.GetProperty().SetOpacity(0.65)
        return actor

    def _build_voxel_actors(
        self, points: np.ndarray, scale: np.ndarray, color: tuple[float, float, float], opacity: float
    ) -> tuple[vtk.vtkActor, vtk.vtkActor]:
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)

        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(polydata)
        glyph.SetSourceConnection(cube.GetOutputPort())
        glyph.ScalingOff()

        actor = vtk.vtkActor()
        actor.SetMapper(glyph)
        actor.GetProperty().SetColor(*color)
        actor.GetProperty().SetOpacity(opacity)
        actor.GetProperty().SetInterpolationToFlat()

        edge_cube = vtk.vtkCubeSource()
        edge_cube.SetXLength(float(scale[0]))
        edge_cube.SetYLength(float(scale[1]))
        edge_cube.SetZLength(float(scale[2]))
        edge_extract = vtk.vtkExtractEdges()
        edge_extract.SetInputConnection(edge_cube.GetOutputPort())
        edge_glyph = vtk.vtkGlyph3DMapper()
        edge_glyph.SetInputData(polydata)
        edge_glyph.SetSourceConnection(edge_extract.GetOutputPort())
        edge_glyph.ScalingOff()

        edge_actor = vtk.vtkActor()
        edge_actor.SetMapper(edge_glyph)
        edge_actor.GetProperty().SetColor(0.0, 0.0, 0.0)
        edge_actor.GetProperty().SetLineWidth(1.0)
        edge_actor.GetProperty().SetOpacity(1.0)
        return actor, edge_actor

    def _refresh_layers(self, checked: bool | None = None) -> None:
        del checked
        for actor, edge_actor in self._layer_actors.values():
            self._renderer.RemoveActor(actor)
            if edge_actor is not None:
                self._renderer.RemoveActor(edge_actor)
        self._layer_actors.clear()

        layer_visibility = {
            "occupied": self.occupied_checkbox.isChecked(),
            "preblocked": self.preblocked_checkbox.isChecked(),
            "traversable": self.traversable_checkbox.isChecked(),
            "risk": self.risk_checkbox.isChecked(),
        }

        for layer_name, visible in layer_visibility.items():
            if not visible or layer_name not in self._layer_data:
                continue
            if layer_name == "risk":
                points, scale, intensity = self._layer_data[layer_name]
                if points.size == 0:
                    continue
                actor, edge_actor = self._build_risk_actors(points, scale, intensity)
            else:
                points, scale, color, opacity = self._layer_data[layer_name]
                if points.size == 0:
                    continue
                actor, edge_actor = self._build_voxel_actors(points, scale, color, opacity)
            self._renderer.AddActor(actor)
            if edge_actor is not None:
                self._renderer.AddActor(edge_actor)
            self._layer_actors[layer_name] = (actor, edge_actor)

        if not self._camera_initialized and self._layer_actors:
            self._renderer.ResetCamera()
            self._camera_initialized = True
        self.vtk_widget.GetRenderWindow().Render()

    def _build_risk_actors(
        self, points: np.ndarray, scale: np.ndarray, intensity: np.ndarray
    ) -> tuple[vtk.vtkActor, None]:
        vtk_points = vtk.vtkPoints()
        vtk_points.SetData(numpy_support.numpy_to_vtk(points.astype(np.float32), deep=True))
        polydata = vtk.vtkPolyData()
        polydata.SetPoints(vtk_points)

        alphas = np.clip(0.12 + 0.83 * intensity.astype(np.float32), 0.12, 0.95)
        colors = np.zeros((len(points), 4), dtype=np.uint8)
        colors[:, 0] = int(0.15 * 255.0)
        colors[:, 1] = int(0.35 * 255.0)
        colors[:, 2] = int(1.0 * 255.0)
        colors[:, 3] = np.round(alphas * 255.0).astype(np.uint8)
        vtk_colors = numpy_support.numpy_to_vtk(colors, deep=True, array_type=vtk.VTK_UNSIGNED_CHAR)
        vtk_colors.SetName("risk_rgba")
        polydata.GetPointData().SetScalars(vtk_colors)

        cube = vtk.vtkCubeSource()
        cube.SetXLength(float(scale[0]))
        cube.SetYLength(float(scale[1]))
        cube.SetZLength(float(scale[2]))
        glyph = vtk.vtkGlyph3DMapper()
        glyph.SetInputData(polydata)
        glyph.SetSourceConnection(cube.GetOutputPort())
        glyph.ScalingOff()
        glyph.SetScalarModeToUsePointData()
        glyph.ScalarVisibilityOn()
        glyph.SetColorModeToDirectScalars()

        actor = vtk.vtkActor()
        actor.SetMapper(glyph)
        actor.GetProperty().SetOpacity(1.0)
        actor.GetProperty().SetInterpolationToFlat()
        return actor, None

    def _update_point_actor(self, kind: str, xyz: tuple[float, float, float]) -> None:
        old_actor = self._start_actor if kind == "start" else self._goal_actor
        if old_actor is not None:
            self._renderer.RemoveActor(old_actor)
        sphere = vtk.vtkSphereSource()
        sphere.SetCenter(*xyz)
        sphere.SetRadius(0.16)
        sphere.SetThetaResolution(18)
        sphere.SetPhiResolution(18)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        if kind == "start":
            actor.GetProperty().SetColor(0.1, 0.95, 0.1)
            self._start_actor = actor
        else:
            actor.GetProperty().SetColor(0.95, 0.1, 0.1)
            self._goal_actor = actor
        actor.GetProperty().SetOpacity(1.0)
        self._renderer.AddActor(actor)
        self.vtk_widget.GetRenderWindow().Render()

    def _update_goal_visual(self, xyz: tuple[float, float, float], yaw: float) -> None:
        self._update_point_actor("goal", xyz)
        if self._goal_arrow_actor is not None:
            self._renderer.RemoveActor(self._goal_arrow_actor)
            self._goal_arrow_actor = None
        arrow = vtk.vtkArrowSource()
        arrow.SetTipResolution(24)
        arrow.SetShaftResolution(24)
        arrow.SetTipLength(0.30)
        arrow.SetTipRadius(0.18)
        arrow.SetShaftRadius(0.08)
        transform = vtk.vtkTransform()
        transform.PostMultiply()
        transform.Scale(0.90, 0.90, 0.90)
        transform.RotateZ(float(np.degrees(yaw)))
        transform.Translate(float(xyz[0]), float(xyz[1]), float(xyz[2]))
        transform_filter = vtk.vtkTransformPolyDataFilter()
        transform_filter.SetTransform(transform)
        transform_filter.SetInputConnection(arrow.GetOutputPort())
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(transform_filter.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.95, 0.1, 0.1)
        actor.GetProperty().SetOpacity(0.95)
        self._renderer.AddActor(actor)
        self._goal_arrow_actor = actor
        self.vtk_widget.GetRenderWindow().Render()

    def _update_path(self, path_points: list[tuple[float, float, float]]) -> None:
        if self._path_actor is not None:
            self._renderer.RemoveActor(self._path_actor)
            self._path_actor = None
        if len(path_points) < 2:
            self.vtk_widget.GetRenderWindow().Render()
            return
        vtk_points = vtk.vtkPoints()
        for point in path_points:
            vtk_points.InsertNextPoint(point)
        poly_line = vtk.vtkPolyLine()
        poly_line.GetPointIds().SetNumberOfIds(len(path_points))
        for i in range(len(path_points)):
            poly_line.GetPointIds().SetId(i, i)
        cells = vtk.vtkCellArray()
        cells.InsertNextCell(poly_line)
        poly_data = vtk.vtkPolyData()
        poly_data.SetPoints(vtk_points)
        poly_data.SetLines(cells)
        tube = vtk.vtkTubeFilter()
        tube.SetInputData(poly_data)
        tube.SetRadius(0.06)
        tube.SetNumberOfSides(16)
        tube.CappingOn()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(tube.GetOutputPort())
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(0.69, 0.40, 1.0)
        actor.GetProperty().SetOpacity(1.0)
        self._renderer.AddActor(actor)
        self._path_actor = actor
        self.vtk_widget.GetRenderWindow().Render()


def main() -> None:
    rclpy.init()
    app = QApplication(sys.argv)
    window = WorldSelectorWindow()
    window.show()
    app.exec_()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
