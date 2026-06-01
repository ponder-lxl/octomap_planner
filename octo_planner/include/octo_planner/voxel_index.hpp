#ifndef OCTO_PLANNER__VOXEL_INDEX_HPP_
#define OCTO_PLANNER__VOXEL_INDEX_HPP_

#include <cstddef>
#include <functional>

namespace octo_planner
{

struct VoxelIndex
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash
{
  std::size_t operator()(const VoxelIndex & k) const
  {
    const std::size_t h1 = std::hash<int>{}(k.x);
    const std::size_t h2 = std::hash<int>{}(k.y);
    const std::size_t h3 = std::hash<int>{}(k.z);
    return h1 ^ (h2 << 1) ^ (h3 << 2);
  }
};

}  // namespace octo_planner

#endif  // OCTO_PLANNER__VOXEL_INDEX_HPP_
