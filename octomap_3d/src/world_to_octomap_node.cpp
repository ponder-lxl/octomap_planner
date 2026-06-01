#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "world_sdf_loader.hpp"

class WorldToOctomapNode : public rclcpp::Node
{
public:
  WorldToOctomapNode()
  : Node("world_to_octomap")
  {
    declare_parameter<std::string>("world_file", "");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<double>("xy_window_size_m", 11.0);
    declare_parameter<double>("ground_surface_max_thickness_m", 0.6);
    declare_parameter<bool>("enable_stair_step_surface_mode", true);
    declare_parameter<double>("stair_step_max_height_m", 0.5);
    declare_parameter<double>("stair_step_max_depth_m", 0.8);
    declare_parameter<double>("stair_step_min_width_m", 1.0);
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("marker_topic", "/octomap_occupied_markers");
    declare_parameter<std::string>("world_file_cmd_topic", "/world_file_cmd");
    declare_parameter<std::vector<std::string>>("gazebo_model_paths", std::vector<std::string>());
    declare_parameter<double>("world_correction_roll", 0.0);
    declare_parameter<double>("world_correction_pitch", 0.0);
    declare_parameter<double>("world_correction_yaw", 0.0);

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("marker_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    world_file_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("world_file_cmd_topic").as_string(), rclcpp::QoS(1).reliable(),
      std::bind(&WorldToOctomapNode::onWorldFileCmd, this, std::placeholders::_1));

    rebuildVoxelizer();

    const auto world_file = get_parameter("world_file").as_string();
    if (!world_file.empty()) {
      loadWorld(world_file);
    } else {
      RCLCPP_WARN(get_logger(), "No initial world_file set. Waiting for /world_file_cmd.");
    }
    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&WorldToOctomapNode::publishAll, this));
  }

private:
  void rebuildVoxelizer()
  {
    WorldSdfVoxelizer::Options options;
    options.resolution = get_parameter("resolution").as_double();
    options.xy_window_size_m = get_parameter("xy_window_size_m").as_double();
    options.ground_surface_max_thickness_m =
      get_parameter("ground_surface_max_thickness_m").as_double();
    options.enable_stair_step_surface_mode =
      get_parameter("enable_stair_step_surface_mode").as_bool();
    options.stair_step_max_height_m = get_parameter("stair_step_max_height_m").as_double();
    options.stair_step_max_depth_m = get_parameter("stair_step_max_depth_m").as_double();
    options.stair_step_min_width_m = get_parameter("stair_step_min_width_m").as_double();
    options.extra_model_paths = get_parameter("gazebo_model_paths").as_string_array();
    options.world_correction_roll = get_parameter("world_correction_roll").as_double();
    options.world_correction_pitch = get_parameter("world_correction_pitch").as_double();
    options.world_correction_yaw = get_parameter("world_correction_yaw").as_double();
    voxelizer_ = std::make_unique<WorldSdfVoxelizer>(options);
  }

  void onWorldFileCmd(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string file = msg->data;
    if (file.empty()) {
      return;
    }
    if (file == loaded_world_file_) {
      RCLCPP_INFO(get_logger(), "Reload requested for same world file: %s", file.c_str());
    }
    loadWorld(file);
  }

  void loadWorld(const std::string & world_file)
  {
    const auto started = std::chrono::steady_clock::now();
    const double resolution = get_parameter("resolution").as_double();
    const double xy_window = get_parameter("xy_window_size_m").as_double();
    const double corr_roll = get_parameter("world_correction_roll").as_double();
    const double corr_pitch = get_parameter("world_correction_pitch").as_double();
    const double corr_yaw = get_parameter("world_correction_yaw").as_double();
    RCLCPP_INFO(
      get_logger(),
      "World->OctoMap conversion STARTED: file=%s resolution=%.3f xy_window=%.1fm "
      "correction_rpy=(%.4f, %.4f, %.4f) rad",
      world_file.c_str(), resolution, xy_window, corr_roll, corr_pitch, corr_yaw);

    rebuildVoxelizer();
    try {
      voxelizer_->loadWorldFile(world_file);
      tree_ = voxelizer_->tree();
      loaded_world_file_ = world_file;
      publishAll();
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      RCLCPP_INFO(
        get_logger(),
        "World->OctoMap conversion COMPLETE: file=%s shapes=%d occupied_voxels=%zu "
        "resolution=%.3f xy_window=%.1fm elapsed=%.2fs "
        "(occupied layer published; waiting for planner layers: preblocked/traversable/risk_cost)",
        world_file.c_str(), voxelizer_->shapeCount(), tree_->size(),
        resolution, xy_window, elapsed);
    } catch (const std::exception & e) {
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
      RCLCPP_ERROR(
        get_logger(), "World->OctoMap conversion FAILED after %.2fs: %s",
        elapsed, e.what());
    }
  }

  void publishAll()
  {
    if (!tree_) {
      return;
    }
    const auto stamp = now();
    const std::string frame_id = get_parameter("frame_id").as_string();

    octomap_msgs::msg::Octomap map_msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, map_msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to serialize generated OctoMap.");
      return;
    }
    map_msg.header.stamp = stamp;
    map_msg.header.frame_id = frame_id;
    octomap_pub_->publish(map_msg);

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = tree_->getResolution();
    marker.scale.y = tree_->getResolution();
    marker.scale.z = tree_->getResolution();
    marker.color.r = 0.95F;
    marker.color.g = 0.45F;
    marker.color.b = 0.15F;
    marker.color.a = 0.95F;

    marker.points.reserve(tree_->size());
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (!tree_->isNodeOccupied(*it)) {
        continue;
      }
      geometry_msgs::msg::Point p;
      p.x = it.getX();
      p.y = it.getY();
      p.z = it.getZ();
      marker.points.push_back(p);
    }
    marker_pub_->publish(marker);
  }

  std::unique_ptr<WorldSdfVoxelizer> voxelizer_;
  std::shared_ptr<octomap::OcTree> tree_;
  std::string loaded_world_file_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr world_file_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorldToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
