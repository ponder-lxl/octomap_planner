#include "octo_planner/path_replan_trigger.hpp"

#include <algorithm>

namespace octo_planner
{

void PathReplanTrigger::configure(const PathReplanTriggerConfig & config)
{
  config_ = config;
  reset();
}

void PathReplanTrigger::reset()
{
  blocked_active_ = false;
  has_blocked_since_ = false;
  has_last_clear_blocked_ = false;
  has_last_fire_ = false;
}

void PathReplanTrigger::update(const bool path_blocked, const rclcpp::Time & now)
{
  if (!config_.enabled) {
    blocked_active_ = false;
    has_blocked_since_ = false;
    return;
  }

  if (path_blocked) {
    if (!blocked_active_) {
      blocked_active_ = true;
      blocked_since_ = now;
      has_blocked_since_ = true;
    }
    has_last_clear_blocked_ = false;
    return;
  }

  if (!blocked_active_) {
    return;
  }

  if (!has_last_clear_blocked_) {
    last_clear_blocked_time_ = now;
    has_last_clear_blocked_ = true;
    return;
  }

  const double grace = std::max(0.0, config_.blocked_clear_grace_sec);
  if ((now - last_clear_blocked_time_).seconds() >= grace) {
    blocked_active_ = false;
    has_blocked_since_ = false;
    has_last_clear_blocked_ = false;
  }
}

double PathReplanTrigger::blockedHoldProgressSec(const rclcpp::Time & now) const
{
  if (!blocked_active_ || !has_blocked_since_) {
    return 0.0;
  }
  return (now - blocked_since_).seconds();
}

bool PathReplanTrigger::isReadyToFire(const rclcpp::Time & now) const
{
  if (!config_.enabled || !blocked_active_ || !has_blocked_since_) {
    return false;
  }
  const double hold = std::max(0.0, config_.blocked_hold_sec);
  if ((now - blocked_since_).seconds() < hold) {
    return false;
  }
  if (has_last_fire_ && config_.cooldown_sec > 0.0) {
    if ((now - last_fire_time_).seconds() < config_.cooldown_sec) {
      return false;
    }
  }
  return true;
}

bool PathReplanTrigger::consumeFire(const rclcpp::Time & now)
{
  if (!isReadyToFire(now)) {
    return false;
  }
  last_fire_time_ = now;
  has_last_fire_ = true;
  return true;
}

}  // namespace octo_planner
