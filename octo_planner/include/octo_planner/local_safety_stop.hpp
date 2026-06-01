#ifndef OCTO_PLANNER__LOCAL_SAFETY_STOP_HPP_
#define OCTO_PLANNER__LOCAL_SAFETY_STOP_HPP_

#include <cstdint>
#include <limits>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace octo_planner
{

enum class LocalSafetyState : std::uint8_t
{
  Disabled = 0,
  Clear,
  Slow,
  Stop,
  StaleCloud,
};

struct LocalSafetyStopConfig
{
  bool enabled{false};
  double stop_distance{0.50};
  double slow_distance{0.80};
  /// body 系水平净空：hypot(x,y) - robot_inscribed_radius（转弯侧向障碍也能触发）
  bool use_horizontal_clearance{true};
  double robot_inscribed_radius{0.12};
  double check_min_x{0.0};
  double check_max_range_xy{2.0};
  double check_half_width{0.35};
  double check_min_z{-0.15};
  double check_max_z{0.85};
  /// 净空小于此值的点视为车体自身，忽略
  double min_clearance_ignore{0.03};
  int min_points_to_trigger{3};
  /// 净空已进入 slow 区时，至少多少点即可触发（稀疏点云仍要减速）
  int min_points_when_close{1};
  /// 点云瞬时变稀疏时，在此时间内仍沿用最近一次最小净空
  double clearance_hold_sec{0.25};
  double cloud_timeout_sec{0.5};
  bool stop_on_stale_cloud{true};
  bool stop_angular_on_stop{true};
  /// SLOW 区同步限制角速度（转弯扫障）
  bool limit_angular_in_slow_zone{true};
  double slow_scale_min{0.15};
  /// 车体参考点到最前端的距离（与 robot_inscribed_radius 一起用于前向净空）
  double robot_forward_margin{0.22};
  /// 缩放后线速度低于此值则置 0，避免 0.03m/s 爬行
  double zero_linear_threshold{0.08};
  /// 持续处于 SLOW 区超过此时间，且净空仍 <= stop_distance+creep_stop_margin 时强制停车
  double slow_zone_force_stop_sec{0.35};
  double creep_stop_margin{0.12};
};

struct LocalSafetyStopStatus
{
  LocalSafetyState state{LocalSafetyState::Disabled};
  double min_obstacle_distance{std::numeric_limits<double>::infinity()};
  int obstacle_points{0};
  int total_cloud_points{0};
  bool cloud_valid{false};
  double cloud_age_sec{0.0};
  std::string source_frame;
  std::string check_frame;
};

const char * localSafetyStateToString(LocalSafetyState state);

/// 基于 body 系 3D 点云的局部静态停障：在 d1_controller 发布 cmd_vel 前调用 filterCmdVel。
class LocalSafetyStop
{
public:
  void configure(const LocalSafetyStopConfig & config);

  void updatePointCloud(
    const sensor_msgs::msg::PointCloud2 & cloud,
    const rclcpp::Time & now,
    const std::string & check_frame = "body");

  geometry_msgs::msg::Twist filterCmdVel(
    const geometry_msgs::msg::Twist & cmd_in,
    const rclcpp::Time & now,
    LocalSafetyStopStatus * status_out = nullptr) const;

  LocalSafetyStopStatus evaluate(const rclcpp::Time & now) const;

  /// 重规划绕障期间临时放宽停障（缩短 stop 距离、禁用爬行强制停车）。
  void beginTemporaryRelax(
    const rclcpp::Time & now,
    double duration_sec,
    double relaxed_stop_distance,
    bool disable_creep_force_stop = true);

  bool isTemporaryRelaxActive(const rclcpp::Time & now) const;

  const LocalSafetyStopConfig & config() const {return config_;}

private:
  bool pointCloudHasField(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const;

  double pointClearance(double x, double y) const;

  double forwardClearance(double forward_x) const;

  double effectiveClearance(const rclcpp::Time & now) const;

  double maxEvalRawRange() const;

  double slowScaleRatio(double clearance, const rclcpp::Time & now) const;

  void updateSlowZoneTimer(LocalSafetyState state, const rclcpp::Time & now) const;

  bool shouldForceStopFromSlowTimer(const rclcpp::Time & now) const;

  void applyFullStop(geometry_msgs::msg::Twist & cmd_out, bool has_angular) const;

  void clampCreepVelocity(geometry_msgs::msg::Twist & cmd_out) const;

  double effectiveStopDistance(const rclcpp::Time & now) const;

  bool creepForceStopDisabled(const rclcpp::Time & now) const;

  LocalSafetyStopConfig config_;
  bool relax_active_{false};
  rclcpp::Time relax_until_;
  double relax_stop_distance_{0.30};
  bool relax_disable_creep_force_stop_{true};
  bool has_cloud_{false};
  rclcpp::Time last_cloud_stamp_;
  std::string last_cloud_frame_;
  double last_min_distance_{std::numeric_limits<double>::infinity()};
  double last_min_forward_x_{std::numeric_limits<double>::infinity()};
  int last_obstacle_points_{0};
  int last_total_cloud_points_{0};
  double latched_clearance_{std::numeric_limits<double>::infinity()};
  rclcpp::Time latch_stamp_;
  mutable bool slow_zone_active_{false};
  mutable rclcpp::Time slow_zone_enter_;
};

}  // namespace octo_planner

#endif  // OCTO_PLANNER__LOCAL_SAFETY_STOP_HPP_
