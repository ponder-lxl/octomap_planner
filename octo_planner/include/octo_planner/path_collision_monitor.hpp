#ifndef OCTO_PLANNER__PATH_COLLISION_MONITOR_HPP_
#define OCTO_PLANNER__PATH_COLLISION_MONITOR_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace octo_planner
{

struct PathCollisionMonitorConfig
{
  bool enabled{false};
  /// 车体等效半径 (m)
  double robot_radius{0.22};
  /// 与 local_safety_stop_distance 对齐；碰撞膨胀半径 = robot_radius + stop_distance
  double stop_distance{0.50};
  /// 相对路径采样点 z 的高度带 [z + min_z, z + max_z]
  double check_min_z{-0.15};
  double check_max_z{0.85};
  /// 沿路径弧长采样间隔 (m)
  double path_sample_step{0.15};
  /// 从机器人最近路径点起，向前最多检查的弧长 (m)
  double max_check_distance{2.0};
  /// 某采样圆柱内至少多少点才判为碰撞
  int min_points_per_sample{2};
  /// 距采样中心小于此半径的点视为噪声/车体，忽略
  double min_point_radius_ignore{0.03};
};

struct RobotPoseMap
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct PathCollisionResult
{
  bool blocked{false};
  /// 碰撞所在路径采样段索引（0 = 离机器人最近的前向采样点）
  int sample_index{-1};
  /// 对应全局路径 pose 索引（路径点下标）
  int path_pose_index{-1};
  /// 沿路径从机器人到碰撞点的弧长 (m)
  double distance_along_path_m{std::numeric_limits<double>::infinity()};
  /// 碰撞采样中心在 map 系
  double collision_x{0.0};
  double collision_y{0.0};
  double collision_z{0.0};
  /// 碰撞中心在机器人 body 系：前向 x、左向 y (m)
  double body_forward_m{std::numeric_limits<double>::infinity()};
  double body_lateral_m{std::numeric_limits<double>::infinity()};
  double inflation_radius_m{0.0};
  int obstacle_points{0};
  bool cloud_valid{false};
};

/// 沿 /planned_path 前方采样，用圆柱（XY 半径 = robot_radius + stop_distance）做点云碰撞检测。
class PathCollisionMonitor
{
public:
  void configure(const PathCollisionMonitorConfig & config);

  void updatePath(const std::vector<geometry_msgs::msg::PoseStamped> & path);

  void updatePointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const rclcpp::Time & now,
    const std::string & check_frame = "map");

  /// @param path_start_index 从该路径点开始向前检查（通常为当前跟踪点 index）
  PathCollisionResult checkCollision(
    const RobotPoseMap & robot_pose,
    int path_start_index,
    const rclcpp::Time & now) const;

  const PathCollisionMonitorConfig & config() const {return config_;}

  /// 膨胀半径 = robot_radius + stop_distance
  double inflationRadius() const;

  /// 供调用方写日志的简要描述
  std::string formatCollisionLog(const PathCollisionResult & result) const;

private:
  struct PathSample
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double arc_length_from_robot{0.0};
    int source_pose_index{0};
  };

  bool pointCloudHasField(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const;

  void buildPathSamples(
    const RobotPoseMap & robot_pose,
    int path_start_index,
    std::vector<PathSample> & samples) const;

  bool sampleCollides(
    const PathSample & sample,
    const sensor_msgs::msg::PointCloud2 & cloud,
    int & obstacle_points_out) const;

  void mapToBody(
    const RobotPoseMap & robot_pose,
    double map_x, double map_y,
    double & body_x, double & body_y) const;

  PathCollisionMonitorConfig config_;
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  bool has_cloud_{false};
  rclcpp::Time last_cloud_stamp_;
  sensor_msgs::msg::PointCloud2 last_cloud_;
  std::string last_cloud_frame_;
};

}  // namespace octo_planner

#endif  // OCTO_PLANNER__PATH_COLLISION_MONITOR_HPP_
