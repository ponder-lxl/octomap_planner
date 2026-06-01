#!/usr/bin/env python3
"""Broadcast TF from nav_msgs/Odometry (e.g. FAST_LIO relocalization on /localization)."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import tf2_ros


class LocalizationOdomTfBroadcaster(Node):
    def __init__(self):
        super().__init__('localization_odom_tf_broadcaster')
        self.declare_parameter('odometry_topic', '/localization')
        self.declare_parameter('override_child_frame', '')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self._override_child = self.get_parameter('override_child_frame').get_parameter_value().string_value
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._sub = self.create_subscription(Odometry, topic, self._cb, qos)
        self.get_logger().info(f'Subscribing {topic} (Odometry -> TF)')

    def _cb(self, msg: Odometry):
        if not msg.header.frame_id:
            self.get_logger().warn_throttle(5000, 'Odometry header.frame_id empty, skip TF')
            return
        child = self._override_child.strip() if self._override_child else msg.child_frame_id
        if not child:
            self.get_logger().warn_throttle(5000, 'child_frame_id empty, skip TF')
            return
        t = TransformStamped()
        t.header = msg.header
        if t.header.stamp.sec == 0 and t.header.stamp.nanosec == 0:
            t.header.stamp = self.get_clock().now().to_msg()
        t.child_frame_id = child
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = LocalizationOdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
