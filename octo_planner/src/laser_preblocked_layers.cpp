#include "octo_planner/laser_preblocked_layers.hpp"

#include <algorithm>
#include <cmath>

namespace octo_planner
{

void LaserPreblockedLayers::configure(const LaserPreblockedLayersConfig & config)
{
  config_ = config;
  if (config_.accumulate_frames < 1) {
    config_.accumulate_frames = 1;
  }
}

void LaserPreblockedLayers::clearAll()
{
  package_cells_.clear();
  history_.clear();
  laser_union_.clear();
}

void LaserPreblockedLayers::clearLaserHistory()
{
  history_.clear();
  laser_union_.clear();
}

void LaserPreblockedLayers::setPackageCells(
  const std::unordered_set<VoxelIndex, VoxelIndexHash> & cells)
{
  package_cells_ = cells;
  if (config_.clear_laser_on_map_import) {
    clearLaserHistory();
  }
}

void LaserPreblockedLayers::setPackageCellsFromWorldPoints(
  const std::vector<geometry_msgs::msg::Point> & points,
  const double resolution)
{
  std::unordered_set<VoxelIndex, VoxelIndexHash> cells;
  const double r = resolution > 0.0 ? resolution : config_.voxel_resolution;
  for (const auto & p : points) {
    cells.insert(worldToVoxel(p.x, p.y, p.z, r));
  }
  setPackageCells(cells);
}

VoxelIndex LaserPreblockedLayers::worldToVoxel(
  const double x, const double y, const double z, const double resolution)
{
  const double r = std::max(1.0e-6, resolution);
  return VoxelIndex{
    static_cast<int>(std::floor(x / r)),
    static_cast<int>(std::floor(y / r)),
    static_cast<int>(std::floor(z / r))};
}

void LaserPreblockedLayers::voxelToWorldCenter(
  const VoxelIndex & idx, const double resolution,
  double & x, double & y, double & z)
{
  const double r = std::max(1.0e-6, resolution);
  x = (static_cast<double>(idx.x) + 0.5) * r;
  y = (static_cast<double>(idx.y) + 0.5) * r;
  z = (static_cast<double>(idx.z) + 0.5) * r;
}

std::unordered_set<VoxelIndex, VoxelIndexHash> LaserPreblockedLayers::inflateCells(
  const std::unordered_set<VoxelIndex, VoxelIndexHash> & seeds,
  const int radius_cells)
{
  std::unordered_set<VoxelIndex, VoxelIndexHash> out;
  const int n = std::max(0, radius_cells);
  for (const auto & seed : seeds) {
    for (int dx = -n; dx <= n; ++dx) {
      for (int dy = -n; dy <= n; ++dy) {
        for (int dz = -n; dz <= n; ++dz) {
          out.insert(VoxelIndex{seed.x + dx, seed.y + dy, seed.z + dz});
        }
      }
    }
  }
  return out;
}

std::size_t LaserPreblockedLayers::pushSnapshotFromWorldCenters(
  const std::vector<geometry_msgs::msg::Point> & cell_centers,
  const double resolution)
{
  const double r = resolution > 0.0 ? resolution : config_.voxel_resolution;
  std::unordered_set<VoxelIndex, VoxelIndexHash> frame;
  for (const auto & p : cell_centers) {
    frame.insert(worldToVoxel(p.x, p.y, p.z, r));
  }

  if (config_.inflate_by_robot_radius && r > 0.0) {
    const int n = std::max(
      1, static_cast<int>(std::ceil(config_.robot_radius / r)));
    frame = inflateCells(frame, n);
  }

  history_.push_back(std::move(frame));
  const std::size_t k = static_cast<std::size_t>(config_.accumulate_frames);
  while (history_.size() > k) {
    history_.pop_front();
  }

  recomputeLaserUnion();
  return laser_union_.size();
}

void LaserPreblockedLayers::recomputeLaserUnion()
{
  laser_union_.clear();
  for (const auto & frame : history_) {
    for (const auto & c : frame) {
      laser_union_.insert(c);
    }
  }
}

std::size_t LaserPreblockedLayers::historyFrameCount() const
{
  return history_.size();
}

std::size_t LaserPreblockedLayers::laserCellCount() const
{
  return laser_union_.size();
}

std::size_t LaserPreblockedLayers::packageCellCount() const
{
  return package_cells_.size();
}

const std::unordered_set<VoxelIndex, VoxelIndexHash> & LaserPreblockedLayers::packageCells() const
{
  return package_cells_;
}

const std::unordered_set<VoxelIndex, VoxelIndexHash> & LaserPreblockedLayers::laserCellsUnion() const
{
  return laser_union_;
}

void LaserPreblockedLayers::rebuildEffectivePreblocked(
  std::unordered_set<VoxelIndex, VoxelIndexHash> & effective_out,
  const std::function<bool(const VoxelIndex &)> & keep_cell) const
{
  effective_out.clear();
  auto insert_if_keep = [&](const VoxelIndex & idx) {
      if (!keep_cell || keep_cell(idx)) {
        effective_out.insert(idx);
      }
    };

  for (const auto & c : package_cells_) {
    insert_if_keep(c);
  }
  for (const auto & c : laser_union_) {
    insert_if_keep(c);
  }
}

}  // namespace octo_planner
