#include "octo_planner/path_collision_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace octo_planner
{

void PathCollisionMonitor::configure(const PathCollisionMonitorConfig & config)
{
  config_ = config;
  path_.clear();
  has_cloud_ = false;
  last_cloud_ = sensor_msgs::msg::PointCloud2();
  last_cloud_frame_.clear();
}

double PathCollisionMonitor::inflationRadius() const
{
  return config_.robot_radius + config_.stop_distance;
}

void PathCollisionMonitor::updatePath(const std::vector<geometry_msgs::msg::PoseStamped> & path)
{
  path_ = path;
}

void PathCollisionMonitor::updatePointCloud(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const rclcpp::Time & now,
  const std::string & check_frame)
{
  if (!config_.enabled) {
    return;
  }

  last_cloud_stamp_ = now;
  last_cloud_ = cloud;
  last_cloud_frame_ = check_frame.empty() ? cloud.header.frame_id : check_frame;
  has_cloud_ = cloud.width > 0 && cloud.height > 0 && !cloud.data.empty() &&
    pointCloudHasField(cloud, "x") && pointCloudHasField(cloud, "y") &&
    pointCloudHasField(cloud, "z");
}

bool PathCollisionMonitor::pointCloudHasField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

void PathCollisionMonitor::mapToBody(
  const RobotPoseMap & robot_pose,
  const double map_x, const double map_y,
  double & body_x, double & body_y) const
{
  const double dx = map_x - robot_pose.x;
  const double dy = map_y - robot_pose.y;
  const double cos_yaw = std::cos(robot_pose.yaw);
  const double sin_yaw = std::sin(robot_pose.yaw);
  body_x = cos_yaw * dx + sin_yaw * dy;
  body_y = -sin_yaw * dx + cos_yaw * dy;
}

void PathCollisionMonitor::buildPathSamples(
  const RobotPoseMap & robot_pose,
  const int path_start_index,
  std::vector<PathSample> & samples) const
{
  samples.clear();
  if (path_.empty()) {
    return;
  }

  const int start_idx = std::clamp(path_start_index, 0, static_cast<int>(path_.size()) - 1);
  const double step = std::max(0.05, config_.path_sample_step);
  double path_arc = 0.0;
  double since_last_sample = step;  // 首点立即采样

  auto arcFromRobotXY = [&robot_pose](const double x, const double y) {
      return std::hypot(x - robot_pose.x, y - robot_pose.y);
    };

  auto add_sample = [&](const double x, const double y, const double z, const int pose_idx) {
      if (path_arc > config_.max_check_distance + 1.0e-3) {
        return;
      }
      PathSample s;
      s.x = x;
      s.y = y;
      s.z = z;
      s.arc_length_from_robot = arcFromRobotXY(x, y);
      s.source_pose_index = pose_idx;
      samples.push_back(s);
    };

  for (int i = start_idx; i < static_cast<int>(path_.size()) - 1; ++i) {
    const auto & p0 = path_[static_cast<std::size_t>(i)].pose.position;
    const auto & p1 = path_[static_cast<std::size_t>(i + 1)].pose.position;
    const double seg_dx = p1.x - p0.x;
    const double seg_dy = p1.y - p0.y;
    const double seg_dz = p1.z - p0.z;
    const double seg_len = std::hypot(seg_dx, seg_dy, seg_dz);
    if (seg_len < 1.0e-6) {
      if (i == start_idx) {
        add_sample(p0.x, p0.y, p0.z, i);
      }
      continue;
    }

    double seg_t = 0.0;
    while (seg_t < seg_len - 1.0e-6) {
      const double advance = std::min(step - since_last_sample, seg_len - seg_t);
      seg_t += advance;
      path_arc += advance;
      since_last_sample += advance;

      if (since_last_sample + 1.0e-6 >= step) {
        const double ratio = seg_t / seg_len;
        add_sample(
          p0.x + ratio * seg_dx,
          p0.y + ratio * seg_dy,
          p0.z + ratio * seg_dz,
          i);
        since_last_sample = 0.0;
      }

      if (path_arc > config_.max_check_distance) {
        return;
      }
    }
  }

  const auto & end_pose = path_.back().pose.position;
  if (samples.empty() ||
    std::hypot(samples.back().x - end_pose.x, samples.back().y - end_pose.y) > step * 0.5)
  {
    if (path_arc <= config_.max_check_distance) {
      add_sample(
        end_pose.x, end_pose.y, end_pose.z,
        static_cast<int>(path_.size()) - 1);
    }
  }
}

bool PathCollisionMonitor::sampleCollides(
  const PathSample & sample,
  const sensor_msgs::msg::PointCloud2 & cloud,
  int & obstacle_points_out) const
{
  obstacle_points_out = 0;
  const double radius = inflationRadius();
  const double radius_sq = radius * radius;
  const double ignore_sq = config_.min_point_radius_ignore * config_.min_point_radius_ignore;
  const double z_min = sample.z + config_.check_min_z;
  const double z_max = sample.z + config_.check_max_z;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double px = static_cast<double>(*iter_x);
    const double py = static_cast<double>(*iter_y);
    const double pz = static_cast<double>(*iter_z);

    const double dx = px - sample.x;
    const double dy = py - sample.y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq > radius_sq) {
      continue;
    }
    if (dist_sq < ignore_sq) {
      continue;
    }
    if (pz < z_min || pz > z_max) {
      continue;
    }
    ++obstacle_points_out;
    if (obstacle_points_out >= config_.min_points_per_sample) {
      return true;
    }
  }
  return false;
}

PathCollisionResult PathCollisionMonitor::checkCollision(
  const RobotPoseMap & robot_pose,
  const int path_start_index,
  const rclcpp::Time & /*now*/) const
{
  PathCollisionResult result;
  result.inflation_radius_m = inflationRadius();
  result.cloud_valid = has_cloud_;

  if (!config_.enabled || path_.empty() || !has_cloud_) {
    return result;
  }

  std::vector<PathSample> samples;
  buildPathSamples(robot_pose, path_start_index, samples);

  for (std::size_t i = 0; i < samples.size(); ++i) {
    const auto & sample = samples[i];
    if (sample.arc_length_from_robot > config_.max_check_distance + 1.0e-3) {
      break;
    }

    int pts = 0;
    if (!sampleCollides(sample, last_cloud_, pts)) {
      continue;
    }

    result.blocked = true;
    result.sample_index = static_cast<int>(i);
    result.path_pose_index = sample.source_pose_index;
    result.distance_along_path_m = sample.arc_length_from_robot;
    result.collision_x = sample.x;
    result.collision_y = sample.y;
    result.collision_z = sample.z;
    result.obstacle_points = pts;
    mapToBody(robot_pose, sample.x, sample.y, result.body_forward_m, result.body_lateral_m);
    return result;
  }

  return result;
}

std::string PathCollisionMonitor::formatCollisionLog(const PathCollisionResult & result) const
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2);
  oss << "路径前方发生碰撞：沿路径约 " << result.distance_along_path_m
      << " m 处（车体前向 " << result.body_forward_m
      << " m，横向 " << result.body_lateral_m
      << " m），map 坐标 ("
      << result.collision_x << ", " << result.collision_y << ", " << result.collision_z
      << ")，路径点 index=" << result.path_pose_index
      << "，膨胀半径 " << result.inflation_radius_m
      << " m (robot_radius=" << config_.robot_radius
      << " + stop_distance=" << config_.stop_distance << ")，障碍点数="
      << result.obstacle_points
      << "；等待重规划触发(持续 blocked 后发布 preblocked)";
  return oss.str();
}

}  // namespace octo_planner
