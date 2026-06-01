#ifndef OCTO_PLANNER__LIDAR_PREBLOCKED_BUILDER_HPP_
#define OCTO_PLANNER__LIDAR_PREBLOCKED_BUILDER_HPP_

#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "octo_planner/path_collision_monitor.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace octo_planner
{

struct LidarPreblockedBuilderConfig
{
  double voxel_resolution{0.1};
  double roi_radius_m{0.8};
  double check_min_z{-0.15};
  double check_max_z{0.85};
  int min_points_per_voxel{2};
};

struct LidarPreblockedBuildResult
{
  bool success{false};
  int raw_points_in_roi{0};
  int voxel_cells{0};
  std::vector<geometry_msgs::msg::Point> cell_centers;
};

/// 将 map 系点云在碰撞点附近体素化，生成 CUBE_LIST 用的体素中心（不做车半径膨胀，由 jie_path 统一膨胀）。
class LidarPreblockedBuilder
{
public:
  void configure(const LidarPreblockedBuilderConfig & config);

  LidarPreblockedBuildResult buildSnapshot(
    const sensor_msgs::msg::PointCloud2 & cloud_map,
    const PathCollisionResult & collision) const;

  visualization_msgs::msg::Marker makeCubeListMarker(
    const std::vector<geometry_msgs::msg::Point> & cell_centers,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  const LidarPreblockedBuilderConfig & config() const {return config_;}

private:
  bool pointCloudHasField(
    const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name) const;

  LidarPreblockedBuilderConfig config_;
};

}  // namespace octo_planner

#endif  // OCTO_PLANNER__LIDAR_PREBLOCKED_BUILDER_HPP_
