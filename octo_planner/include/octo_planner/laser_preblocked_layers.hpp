#ifndef OCTO_PLANNER__LASER_PREBLOCKED_LAYERS_HPP_
#define OCTO_PLANNER__LASER_PREBLOCKED_LAYERS_HPP_

#include <cstddef>
#include <deque>
#include <functional>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "octo_planner/voxel_index.hpp"

namespace octo_planner
{

struct LaserPreblockedLayersConfig
{
  int accumulate_frames{5};
  double robot_radius{0.22};
  double voxel_resolution{0.1};
  bool clear_laser_on_map_import{true};
  bool inflate_by_robot_radius{true};
};

/// 地图包 preblocked 与激光 K 帧滑动窗口（方案 A：仅在重规划入队时 push）。
class LaserPreblockedLayers
{
public:
  void configure(const LaserPreblockedLayersConfig & config);

  void clearAll();

  void clearLaserHistory();

  void setPackageCells(const std::unordered_set<VoxelIndex, VoxelIndexHash> & cells);

  void setPackageCellsFromWorldPoints(
    const std::vector<geometry_msgs::msg::Point> & points,
    double resolution);

  /// 将单帧体素中心（map 系）入队；内部体素化、膨胀后写入历史。
  std::size_t pushSnapshotFromWorldCenters(
    const std::vector<geometry_msgs::msg::Point> & cell_centers,
    double resolution);

  std::size_t historyFrameCount() const;

  std::size_t laserCellCount() const;

  std::size_t packageCellCount() const;

  const std::unordered_set<VoxelIndex, VoxelIndexHash> & packageCells() const;

  const std::unordered_set<VoxelIndex, VoxelIndexHash> & laserCellsUnion() const;

  /// effective = package ∪ laser；filter 返回 false 的格不写入（如 occupied）。
  void rebuildEffectivePreblocked(
    std::unordered_set<VoxelIndex, VoxelIndexHash> & effective_out,
    const std::function<bool(const VoxelIndex &)> & keep_cell) const;

  static VoxelIndex worldToVoxel(double x, double y, double z, double resolution);

  static void voxelToWorldCenter(
    const VoxelIndex & idx, double resolution,
    double & x, double & y, double & z);

  static std::unordered_set<VoxelIndex, VoxelIndexHash> inflateCells(
    const std::unordered_set<VoxelIndex, VoxelIndexHash> & seeds,
    int radius_cells);

  const LaserPreblockedLayersConfig & config() const {return config_;}

private:
  void recomputeLaserUnion();

  LaserPreblockedLayersConfig config_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> package_cells_;
  std::deque<std::unordered_set<VoxelIndex, VoxelIndexHash>> history_;
  std::unordered_set<VoxelIndex, VoxelIndexHash> laser_union_;
};

}  // namespace octo_planner

#endif  // OCTO_PLANNER__LASER_PREBLOCKED_LAYERS_HPP_
