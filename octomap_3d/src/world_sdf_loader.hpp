#pragma once

#include <memory>
#include <string>
#include <vector>

#include <octomap/OcTree.h>

class WorldSdfVoxelizer
{
public:
  struct Options
  {
    double resolution{0.2};
    double xy_window_size_m{11.0};
    double ground_surface_max_thickness_m{0.6};
    bool enable_stair_step_surface_mode{true};
    double stair_step_max_height_m{0.5};
    double stair_step_max_depth_m{0.8};
    double stair_step_min_width_m{1.0};
    std::vector<std::string> extra_model_paths;
    double world_correction_roll{0.0};
    double world_correction_pitch{0.0};
    double world_correction_yaw{0.0};
  };

  explicit WorldSdfVoxelizer(Options options);
  ~WorldSdfVoxelizer();

  void loadWorldFile(const std::string & world_file);
  std::shared_ptr<octomap::OcTree> tree() const;
  int shapeCount() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
