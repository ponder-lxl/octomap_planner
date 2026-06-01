#include "octo_planner/lidar_preblocked_builder.hpp"

#include <cmath>
#include <unordered_map>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace octo_planner
{

void LidarPreblockedBuilder::configure(const LidarPreblockedBuilderConfig & config)
{
  config_ = config;
}

bool LidarPreblockedBuilder::pointCloudHasField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return true;
    }
  }
  return false;
}

LidarPreblockedBuildResult LidarPreblockedBuilder::buildSnapshot(
  const sensor_msgs::msg::PointCloud2 & cloud_map,
  const PathCollisionResult & collision) const
{
  LidarPreblockedBuildResult result;
  if (!collision.blocked || config_.voxel_resolution <= 1.0e-6) {
    return result;
  }

  if (!pointCloudHasField(cloud_map, "x") || !pointCloudHasField(cloud_map, "y") ||
    !pointCloudHasField(cloud_map, "z"))
  {
    return result;
  }

  const double r = config_.voxel_resolution;
  const double roi_sq = config_.roi_radius_m * config_.roi_radius_m;
  const double z_min = collision.collision_z + config_.check_min_z;
  const double z_max = collision.collision_z + config_.check_max_z;

  struct VoxelKey
  {
    int x;
    int y;
    int z;
    bool operator==(const VoxelKey & o) const
    {
      return x == o.x && y == o.y && z == o.z;
    }
  };

  struct VoxelKeyHash
  {
    std::size_t operator()(const VoxelKey & k) const
    {
      return (static_cast<std::size_t>(k.x) * 73856093U) ^
             (static_cast<std::size_t>(k.y) * 19349663U) ^
             (static_cast<std::size_t>(k.z) * 83492791U);
    }
  };

  std::unordered_map<VoxelKey, int, VoxelKeyHash> counts;

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_map, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud_map, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud_map, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    const double px = static_cast<double>(*iter_x);
    const double py = static_cast<double>(*iter_y);
    const double pz = static_cast<double>(*iter_z);

    const double dx = px - collision.collision_x;
    const double dy = py - collision.collision_y;
    if (dx * dx + dy * dy > roi_sq) {
      continue;
    }
    if (pz < z_min || pz > z_max) {
      continue;
    }

    ++result.raw_points_in_roi;
    const VoxelKey key{
      static_cast<int>(std::floor(px / r)),
      static_cast<int>(std::floor(py / r)),
      static_cast<int>(std::floor(pz / r))};
    ++counts[key];
  }

  for (const auto & entry : counts) {
    if (entry.second < config_.min_points_per_voxel) {
      continue;
    }
    geometry_msgs::msg::Point p;
    p.x = (static_cast<double>(entry.first.x) + 0.5) * r;
    p.y = (static_cast<double>(entry.first.y) + 0.5) * r;
    p.z = (static_cast<double>(entry.first.z) + 0.5) * r;
    result.cell_centers.push_back(p);
  }

  result.voxel_cells = static_cast<int>(result.cell_centers.size());
  result.success = !result.cell_centers.empty();
  return result;
}

visualization_msgs::msg::Marker LidarPreblockedBuilder::makeCubeListMarker(
  const std::vector<geometry_msgs::msg::Point> & cell_centers,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  visualization_msgs::msg::Marker msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  msg.ns = "external_preblocked_cells";
  msg.id = 0;
  msg.type = visualization_msgs::msg::Marker::CUBE_LIST;
  msg.action = visualization_msgs::msg::Marker::ADD;
  msg.pose.orientation.w = 1.0;
  msg.scale.x = config_.voxel_resolution;
  msg.scale.y = config_.voxel_resolution;
  msg.scale.z = config_.voxel_resolution;
  msg.color.r = 0.95F;
  msg.color.g = 0.10F;
  msg.color.b = 0.10F;
  msg.color.a = 0.95F;
  msg.points = cell_centers;
  return msg;
}

}  // namespace octo_planner
