#!/usr/bin/env python3
"""
RViz 导航目标桥接：将 RViz 交互转为 jie_path_node 所需的 start / goal_point / goal_pose。

支持两种输入（由 goal_input 选择）：
- clicked_point：RViz「Publish Point」-> /clicked_point，终点使用点击的 (x,y,z)
- goal_pose：RViz「2D Goal Pose」-> /goal_pose，终点 xy 来自 RViz，z 取车辆当前高度

起点均为 map 下当前 base_frame 位姿（TF）。
发布顺序：start -> goal_pose -> goal_point。
同时在 /nav_bridge/selection_markers 发布起点/终点可视化 Marker。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
from visualization_msgs.msg import Marker, MarkerArray
import tf2_geometry_msgs  # noqa: F401  registers PointStamped for tf2
from tf2_geometry_msgs import do_transform_point
import tf2_ros
from tf2_ros import TransformException


class RvizGoalNavBridge(Node):
    def __init__(self):
        super().__init__('rviz_goal_nav_bridge')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('goal_input', 'clicked_point')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('rviz_goal_topic', '/goal_pose')
        self.declare_parameter('marker_topic', '/nav_bridge/selection_markers')
        self.declare_parameter('start_topic', '/nav_bridge/start')
        self.declare_parameter('goal_point_topic', '/nav_bridge/goal_point')
        self.declare_parameter('goal_pose_topic', '/nav_bridge/goal_pose')
        self.declare_parameter('marker_cube_size', 0.25)
        self.declare_parameter('marker_arrow_length', 0.8)
        self.declare_parameter('marker_arrow_shaft', 0.10)

        self._map_f = self.get_parameter('map_frame').get_parameter_value().string_value
        self._base_f = self.get_parameter('base_frame').get_parameter_value().string_value
        self._goal_input = self.get_parameter('goal_input').get_parameter_value().string_value
        self._cube_size = self.get_parameter('marker_cube_size').get_parameter_value().double_value
        self._arrow_len = self.get_parameter('marker_arrow_length').get_parameter_value().double_value
        self._arrow_shaft = self.get_parameter('marker_arrow_shaft').get_parameter_value().double_value

        st = self.get_parameter('start_topic').get_parameter_value().string_value
        gt = self.get_parameter('goal_point_topic').get_parameter_value().string_value
        gp = self.get_parameter('goal_pose_topic').get_parameter_value().string_value
        marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value

        # 起点/终点为一次性导航指令，用 volatile 避免 latched 旧消息被重复投递。
        nav_goal_pub_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        marker_pub_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub_start = self.create_publisher(PointStamped, st, nav_goal_pub_qos)
        self._pub_goal_pt = self.create_publisher(PointStamped, gt, nav_goal_pub_qos)
        self._pub_goal_pose = self.create_publisher(PoseStamped, gp, nav_goal_pub_qos)
        self._pub_markers = self.create_publisher(MarkerArray, marker_topic, marker_pub_qos)

        self._last_click_ns = 0
        self._click_debounce_sec = 0.4

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        sub_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
        )
        if self._goal_input == 'clicked_point':
            clicked_topic = self.get_parameter('clicked_point_topic').get_parameter_value().string_value
            self.create_subscription(PointStamped, clicked_topic, self._on_clicked_point, sub_qos)
            self.get_logger().info(
                f'Bridge (Publish Point): sub={clicked_topic} -> start={st} goal_pt={gt} '
                f'goal_pose={gp} markers={marker_topic} ({self._map_f} <- {self._base_f})'
            )
        elif self._goal_input == 'goal_pose':
            sub_topic = self.get_parameter('rviz_goal_topic').get_parameter_value().string_value
            self.create_subscription(PoseStamped, sub_topic, self._on_goal_pose, sub_qos)
            self.get_logger().info(
                f'Bridge (2D Goal Pose): sub={sub_topic} -> start={st} goal_pt={gt} '
                f'goal_pose={gp} markers={marker_topic} ({self._map_f} <- {self._base_f})'
            )
        else:
            raise ValueError(f'Unsupported goal_input={self._goal_input!r}, use clicked_point or goal_pose')

    def _lookup_robot_start(self) -> tuple[float, float, float] | None:
        try:
            tf: TransformStamped = self._tf_buffer.lookup_transform(
                self._map_f,
                self._base_f,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as ex:
            self.get_logger().warn(f'TF {self._map_f}->{self._base_f} failed: {ex}')
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y), float(t.z)

    def _to_map_point(self, msg: PointStamped) -> PointStamped | None:
        stamp = self.get_clock().now().to_msg()
        src_frame = msg.header.frame_id if msg.header.frame_id else self._map_f
        if src_frame == self._map_f:
            out = PointStamped()
            out.header.frame_id = self._map_f
            out.header.stamp = stamp
            out.point = msg.point
            return out
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_f,
                src_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
            out = do_transform_point(msg, tf)
            out.header.frame_id = self._map_f
            out.header.stamp = stamp
            return out
        except TransformException as ex:
            self.get_logger().warn(
                f'Transform clicked point {src_frame}->{self._map_f} failed: {ex}'
            )
            return None

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    @staticmethod
    def _quaternion_to_yaw(ox: float, oy: float, oz: float, ow: float) -> float:
        # yaw from quaternion (Z rotation)
        siny_cosp = 2.0 * (ow * oz + ox * oy)
        cosy_cosp = 1.0 - 2.0 * (oy * oy + oz * oz)
        return math.atan2(siny_cosp, cosy_cosp)

    def _make_cube(self, marker_id: int, stamp, x: float, y: float, z: float,
                   r: float, g: float, b: float) -> Marker:
        m = Marker()
        m.header.frame_id = self._map_f
        m.header.stamp = stamp
        m.ns = 'nav_bridge'
        m.id = marker_id
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = self._cube_size
        m.scale.y = self._cube_size
        m.scale.z = self._cube_size
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.95
        return m

    def _make_sphere(self, marker_id: int, stamp, x: float, y: float, z: float,
                     r: float, g: float, b: float) -> Marker:
        m = Marker()
        m.header.frame_id = self._map_f
        m.header.stamp = stamp
        m.ns = 'nav_bridge'
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        d = self._cube_size * 1.2
        m.scale.x = d
        m.scale.y = d
        m.scale.z = d
        m.color.r = r
        m.color.g = g
        m.color.b = b
        m.color.a = 0.95
        return m

    def _make_yaw_arrow(self, marker_id: int, stamp, x: float, y: float, z: float,
                        yaw: float) -> Marker:
        m = Marker()
        m.header.frame_id = self._map_f
        m.header.stamp = stamp
        m.ns = 'nav_bridge'
        m.id = marker_id
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        half = yaw * 0.5
        m.pose.orientation.z = math.sin(half)
        m.pose.orientation.w = math.cos(half)
        m.scale.x = self._arrow_len
        m.scale.y = self._arrow_shaft
        m.scale.z = self._arrow_shaft
        m.color.r = 0.95
        m.color.g = 0.15
        m.color.b = 0.15
        m.color.a = 0.95
        return m

    def _publish_selection_markers(
        self,
        stamp,
        sx: float,
        sy: float,
        sz: float,
        gx: float,
        gy: float,
        gz: float,
        yaw: float,
    ) -> None:
        arr = MarkerArray()
        arr.markers.append(self._make_cube(0, stamp, sx, sy, sz, 0.1, 0.95, 0.1))
        arr.markers.append(self._make_sphere(1, stamp, gx, gy, gz, 0.95, 0.15, 0.15))
        arr.markers.append(self._make_yaw_arrow(2, stamp, gx, gy, gz, yaw))
        self._pub_markers.publish(arr)

    def _publish_nav_goal(
        self,
        gx: float,
        gy: float,
        gz: float,
        goal_frame: str,
        ox: float,
        oy: float,
        oz: float,
        ow: float,
    ) -> None:
        start_xyz = self._lookup_robot_start()
        if start_xyz is None:
            return
        sx, sy, sz = start_xyz
        stamp = self.get_clock().now().to_msg()
        frame = goal_frame if goal_frame else self._map_f

        start = PointStamped()
        start.header.frame_id = self._map_f
        start.header.stamp = stamp
        start.point.x = sx
        start.point.y = sy
        start.point.z = sz

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame
        goal_pose.header.stamp = stamp
        goal_pose.pose.position.x = gx
        goal_pose.pose.position.y = gy
        goal_pose.pose.position.z = gz
        goal_pose.pose.orientation.x = ox
        goal_pose.pose.orientation.y = oy
        goal_pose.pose.orientation.z = oz
        goal_pose.pose.orientation.w = ow

        goal_pt = PointStamped()
        goal_pt.header.frame_id = frame
        goal_pt.header.stamp = stamp
        goal_pt.point.x = gx
        goal_pt.point.y = gy
        goal_pt.point.z = gz

        yaw = self._quaternion_to_yaw(ox, oy, oz, ow)

        self._pub_start.publish(start)
        self._pub_goal_pose.publish(goal_pose)
        self._pub_goal_pt.publish(goal_pt)
        self._publish_selection_markers(stamp, sx, sy, sz, gx, gy, gz, yaw)
        self.get_logger().info(
            f'Nav goal: start=({sx:.2f},{sy:.2f},{sz:.2f}) -> '
            f'goal=({gx:.2f},{gy:.2f},{gz:.2f})'
        )

    def _accept_new_click(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_click_ns < int(self._click_debounce_sec * 1e9):
            self.get_logger().warn('Ignored duplicate click within debounce window.')
            return False
        self._last_click_ns = now_ns
        return True

    def _on_clicked_point(self, msg: PointStamped) -> None:
        if not self._accept_new_click():
            return
        goal_in_map = self._to_map_point(msg)
        if goal_in_map is None:
            return

        gx = float(goal_in_map.point.x)
        gy = float(goal_in_map.point.y)
        gz = float(goal_in_map.point.z)

        start_xyz = self._lookup_robot_start()
        if start_xyz is None:
            return
        sx, sy, _ = start_xyz
        yaw = math.atan2(gy - sy, gx - sx)
        ox, oy, oz, ow = self._yaw_to_quaternion(yaw)

        self._publish_nav_goal(gx, gy, gz, self._map_f, ox, oy, oz, ow)

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        if not self._accept_new_click():
            return
        start_xyz = self._lookup_robot_start()
        if start_xyz is None:
            return
        _, _, sz = start_xyz

        frame = msg.header.frame_id if msg.header.frame_id else self._map_f
        gz = sz
        ox = float(msg.pose.orientation.x)
        oy = float(msg.pose.orientation.y)
        oz = float(msg.pose.orientation.z)
        ow = float(msg.pose.orientation.w)

        self._publish_nav_goal(
            float(msg.pose.position.x),
            float(msg.pose.position.y),
            gz,
            frame,
            ox,
            oy,
            oz,
            ow,
        )


def main():
    rclpy.init()
    node = RvizGoalNavBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
