#include "octo_planner/local_safety_stop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace octo_planner
{

const char * localSafetyStateToString(const LocalSafetyState state)
{
  switch (state) {
    case LocalSafetyState::Disabled:
      return "DISABLED";
    case LocalSafetyState::Clear:
      return "CLEAR";
    case LocalSafetyState::Slow:
      return "SLOW";
    case LocalSafetyState::Stop:
      return "STOP";
    case LocalSafetyState::StaleCloud:
      return "STALE_CLOUD";
    default:
      return "UNKNOWN";
  }
}

void LocalSafetyStop::configure(const LocalSafetyStopConfig & config)
{
  config_ = config;
  has_cloud_ = false;
  last_min_distance_ = std::numeric_limits<double>::infinity();
  last_min_forward_x_ = std::numeric_limits<double>::infinity();
  last_obstacle_points_ = 0;
  last_total_cloud_points_ = 0;
  last_cloud_frame_.clear();
  latched_clearance_ = std::numeric_limits<double>::infinity();
  latch_stamp_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  slow_zone_active_ = false;
  slow_zone_enter_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  relax_active_ = false;
}

void LocalSafetyStop::beginTemporaryRelax(
  const rclcpp::Time & now,
  const double duration_sec,
  const double relaxed_stop_distance,
  const bool disable_creep_force_stop)
{
  if (duration_sec <= 0.0) {
    relax_active_ = false;
    return;
  }
  relax_active_ = true;
  relax_until_ = now + rclcpp::Duration::from_seconds(duration_sec);
  relax_stop_distance_ = std::max(0.05, relaxed_stop_distance);
  relax_disable_creep_force_stop_ = disable_creep_force_stop;
  slow_zone_active_ = false;
}

bool LocalSafetyStop::isTemporaryRelaxActive(const rclcpp::Time & now) const
{
  return relax_active_ && now <= relax_until_;
}

double LocalSafetyStop::effectiveStopDistance(const rclcpp::Time & now) const
{
  if (isTemporaryRelaxActive(now)) {
    return relax_stop_distance_;
  }
  return config_.stop_distance;
}

bool LocalSafetyStop::creepForceStopDisabled(const rclcpp::Time & now) const
{
  return isTemporaryRelaxActive(now) && relax_disable_creep_force_stop_;
}

bool LocalSafetyStop::pointCloudHasField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

double LocalSafetyStop::pointClearance(double x, double y) const
{
  if (config_.use_horizontal_clearance) {
    const double raw_xy = std::hypot(x, y);
    return std::max(0.0, raw_xy - config_.robot_inscribed_radius);
  }
  return std::max(0.0, x - config_.robot_inscribed_radius);
}

double LocalSafetyStop::maxEvalRawRange() const
{
  return config_.slow_distance + config_.robot_inscribed_radius + 0.05;
}

double LocalSafetyStop::forwardClearance(const double forward_x) const
{
  return std::max(0.0, forward_x - config_.robot_forward_margin);
}

double LocalSafetyStop::effectiveClearance(const rclcpp::Time & /*now*/) const
{
  double clearance = last_min_distance_;
  if (!std::isfinite(clearance)) {
    clearance = std::numeric_limits<double>::infinity();
  }

  if (std::isfinite(last_min_forward_x_)) {
    clearance = std::min(clearance, forwardClearance(last_min_forward_x_));
  }
  return clearance;
}

double LocalSafetyStop::slowScaleRatio(
  const double clearance, const rclcpp::Time & now) const
{
  const double stop_dist = effectiveStopDistance(now);
  const double span = std::max(1.0e-3, config_.slow_distance - stop_dist);
  return std::clamp(
    (clearance - stop_dist) / span,
    config_.slow_scale_min, 1.0);
}

void LocalSafetyStop::updatePointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const rclcpp::Time & now,
  const std::string & check_frame)
{
  if (!config_.enabled) {
    return;
  }

  last_cloud_stamp_ = now;
  last_cloud_frame_ = check_frame.empty() ? cloud.header.frame_id : check_frame;
  last_obstacle_points_ = 0;
  last_total_cloud_points_ = 0;
  last_min_distance_ = std::numeric_limits<double>::infinity();
  last_min_forward_x_ = std::numeric_limits<double>::infinity();

  if (cloud.width == 0 || cloud.height == 0 || cloud.data.empty()) {
    has_cloud_ = true;
    return;
  }

  if (!pointCloudHasField(cloud, "x") || !pointCloudHasField(cloud, "y") ||
    !pointCloudHasField(cloud, "z"))
  {
    has_cloud_ = false;
    return;
  }

  has_cloud_ = true;
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    ++last_total_cloud_points_;
    const double x = static_cast<double>(*iter_x);
    const double y = static_cast<double>(*iter_y);
    const double z = static_cast<double>(*iter_z);

    if (x < config_.check_min_x) {
      continue;
    }
    const double range_xy = std::hypot(x, y);
    if (range_xy > config_.check_max_range_xy) {
      continue;
    }
    if (range_xy > maxEvalRawRange()) {
      continue;
    }
    if (std::abs(y) > config_.check_half_width) {
      continue;
    }
    if (z < config_.check_min_z || z > config_.check_max_z) {
      continue;
    }

    const double clearance = pointClearance(x, y);
    if (clearance < config_.min_clearance_ignore) {
      continue;
    }

    ++last_obstacle_points_;
    last_min_forward_x_ = std::min(last_min_forward_x_, x);
    last_min_distance_ = std::min(last_min_distance_, clearance);
  }

  if (last_obstacle_points_ > 0) {
    const double frame_clearance = effectiveClearance(now);
    if (frame_clearance <= config_.slow_distance) {
      if (!std::isfinite(latched_clearance_) || frame_clearance < latched_clearance_) {
        latched_clearance_ = frame_clearance;
        latch_stamp_ = now;
      }
    }
  }
}

void LocalSafetyStop::updateSlowZoneTimer(
  const LocalSafetyState state, const rclcpp::Time & now) const
{
  if (state == LocalSafetyState::Slow) {
    if (!slow_zone_active_) {
      slow_zone_active_ = true;
      slow_zone_enter_ = now;
    }
    return;
  }
  slow_zone_active_ = false;
}

bool LocalSafetyStop::shouldForceStopFromSlowTimer(const rclcpp::Time & now) const
{
  if (!slow_zone_active_ || config_.slow_zone_force_stop_sec <= 0.0) {
    return false;
  }
  return (now - slow_zone_enter_).seconds() >= config_.slow_zone_force_stop_sec;
}

void LocalSafetyStop::applyFullStop(geometry_msgs::msg::Twist & cmd_out, const bool has_angular) const
{
  if (cmd_out.linear.x > 0.0) {
    cmd_out.linear.x = 0.0;
  }
  if (std::abs(cmd_out.linear.y) > 1.0e-6) {
    cmd_out.linear.y = 0.0;
  }
  if (config_.stop_angular_on_stop && has_angular) {
    cmd_out.angular.z = 0.0;
  }
}

void LocalSafetyStop::clampCreepVelocity(geometry_msgs::msg::Twist & cmd_out) const
{
  if (cmd_out.linear.x > 0.0 && cmd_out.linear.x < config_.zero_linear_threshold) {
    cmd_out.linear.x = 0.0;
  }
}

LocalSafetyStopStatus LocalSafetyStop::evaluate(const rclcpp::Time & now) const
{
  LocalSafetyStopStatus status;
  status.source_frame = last_cloud_frame_;
  status.check_frame = last_cloud_frame_;
  status.total_cloud_points = last_total_cloud_points_;
  status.obstacle_points = last_obstacle_points_;
  status.min_obstacle_distance = effectiveClearance(now);

  if (!config_.enabled) {
    status.state = LocalSafetyState::Disabled;
    return status;
  }

  if (!has_cloud_) {
    status.state = config_.stop_on_stale_cloud ?
      LocalSafetyState::StaleCloud : LocalSafetyState::Clear;
    return status;
  }

  status.cloud_age_sec = (now - last_cloud_stamp_).seconds();
  status.cloud_valid = status.cloud_age_sec <= config_.cloud_timeout_sec;
  if (!status.cloud_valid) {
    status.state = config_.stop_on_stale_cloud ?
      LocalSafetyState::StaleCloud : LocalSafetyState::Clear;
    return status;
  }

  const double clearance = status.min_obstacle_distance;
  int required_points = config_.min_points_to_trigger;
  if (std::isfinite(clearance) && clearance <= config_.slow_distance) {
    required_points = std::min(required_points, config_.min_points_when_close);
  }

  const bool latch_active = std::isfinite(latched_clearance_) &&
    (now - latch_stamp_).seconds() <= config_.clearance_hold_sec &&
    latched_clearance_ <= config_.slow_distance;

  const bool enough_points = last_obstacle_points_ >= required_points;

  double clearance_for_state = clearance;
  if (!std::isfinite(clearance_for_state)) {
    if (latch_active) {
      clearance_for_state = latched_clearance_;
    } else {
      status.state = LocalSafetyState::Clear;
      return status;
    }
  } else if (!enough_points && latch_active) {
    clearance_for_state = latched_clearance_;
  }

  if (!enough_points && !latch_active) {
    status.state = LocalSafetyState::Clear;
    return status;
  }

  status.min_obstacle_distance = clearance_for_state;

  const double stop_dist = effectiveStopDistance(now);
  if (clearance_for_state <= stop_dist) {
    status.state = LocalSafetyState::Stop;
    return status;
  }

  if (clearance_for_state <= config_.slow_distance) {
    status.state = LocalSafetyState::Slow;
    return status;
  }

  status.state = LocalSafetyState::Clear;
  return status;
}

geometry_msgs::msg::Twist LocalSafetyStop::filterCmdVel(
  const geometry_msgs::msg::Twist & cmd_in,
  const rclcpp::Time & now,
  LocalSafetyStopStatus * status_out) const
{
  geometry_msgs::msg::Twist cmd_out = cmd_in;
  LocalSafetyStopStatus status = evaluate(now);
  updateSlowZoneTimer(status.state, now);

  if (status.state == LocalSafetyState::Slow && !creepForceStopDisabled(now) &&
    shouldForceStopFromSlowTimer(now))
  {
    const double c = status.min_obstacle_distance;
    const double stop_dist = effectiveStopDistance(now);
    if (std::isfinite(c) && c <= stop_dist + config_.creep_stop_margin) {
      status.state = LocalSafetyState::Stop;
    }
  }

  if (status_out != nullptr) {
    *status_out = status;
  }

  if (!config_.enabled || status.state == LocalSafetyState::Disabled ||
    status.state == LocalSafetyState::Clear)
  {
    return cmd_out;
  }

  const bool block_forward = cmd_out.linear.x > 0.0;
  const bool has_angular = std::abs(cmd_out.angular.z) > 1.0e-4;
  const double clearance = status.min_obstacle_distance;
  const double ratio = slowScaleRatio(clearance, now);

  if (status.state == LocalSafetyState::StaleCloud) {
    applyFullStop(cmd_out, has_angular);
    return cmd_out;
  }

  if (status.state == LocalSafetyState::Stop) {
    applyFullStop(cmd_out, has_angular);
    return cmd_out;
  }

  if (status.state == LocalSafetyState::Slow) {
    if (block_forward) {
      cmd_out.linear.x *= ratio;
    }
    if (config_.limit_angular_in_slow_zone && has_angular) {
      cmd_out.angular.z *= ratio;
    }
    clampCreepVelocity(cmd_out);
    return cmd_out;
  }

  return cmd_out;
}

}  // namespace octo_planner
