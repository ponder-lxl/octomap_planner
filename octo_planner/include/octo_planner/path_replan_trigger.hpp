#ifndef OCTO_PLANNER__PATH_REPLAN_TRIGGER_HPP_
#define OCTO_PLANNER__PATH_REPLAN_TRIGGER_HPP_

#include "rclcpp/time.hpp"

namespace octo_planner
{

struct PathReplanTriggerConfig
{
  bool enabled{false};
  double blocked_hold_sec{1.0};
  double cooldown_sec{5.0};
  /// 连续无 blocked 超过此时间才清零计时，避免路径碰撞单帧抖动
  double blocked_clear_grace_sec{0.25};
};

/// blocked 持续 hold_sec 且距上次触发超过 cooldown 时，consumeFire() 返回 true（方案 A：每次触发入队一帧）。
class PathReplanTrigger
{
public:
  void configure(const PathReplanTriggerConfig & config);

  void reset();

  /// @param path_blocked PathCollisionMonitor::blocked
  void update(bool path_blocked, const rclcpp::Time & now);

  /// 若满足触发条件则返回 true 一次，并记录上次触发时间。
  bool consumeFire(const rclcpp::Time & now);

  bool isBlockedAccumulating() const {return blocked_active_;}

  bool isReadyToFire(const rclcpp::Time & now) const;

  double blockedHoldProgressSec(const rclcpp::Time & now) const;

  const PathReplanTriggerConfig & config() const {return config_;}

private:
  PathReplanTriggerConfig config_;
  bool blocked_active_{false};
  bool has_blocked_since_{false};
  rclcpp::Time blocked_since_;
  rclcpp::Time last_clear_blocked_time_;
  bool has_last_clear_blocked_{false};
  rclcpp::Time last_fire_time_;
  bool has_last_fire_{false};
};

}  // namespace octo_planner

#endif  // OCTO_PLANNER__PATH_REPLAN_TRIGGER_HPP_
