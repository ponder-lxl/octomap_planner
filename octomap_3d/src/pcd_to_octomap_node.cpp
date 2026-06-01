#include <array>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <open3d/Open3D.h>

#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"

class PcdToOctomapNode : public rclcpp::Node
{
public:
  PcdToOctomapNode()
  : Node("pcd_to_octomap")
  {
    declare_parameter<std::string>("pcd_file", "");
    declare_parameter<std::string>("pcd_file_cmd_topic", "/pcd_file_cmd");
    declare_parameter<std::string>("map_cloud_topic", "");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<double>("voxel_downsample_m", 0.0);
    declare_parameter<int>("min_points_per_voxel", 3);
    declare_parameter<int>("min_cluster_voxels", 4);
    // Deprecated compatibility parameters. Kept declared so existing launch files still work.
    declare_parameter<double>("min_z", -1.0e9);
    declare_parameter<double>("max_z", 1.0e9);

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    pcd_file_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("pcd_file_cmd_topic").as_string(), rclcpp::QoS(1).reliable(),
      std::bind(&PcdToOctomapNode::onPcdFileCmd, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&PcdToOctomapNode::publishMap, this));

    map_cloud_topic_ = get_parameter("map_cloud_topic").as_string();
    const auto pcd_file = get_parameter("pcd_file").as_string();

    if (!map_cloud_topic_.empty()) {
      map_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        map_cloud_topic_,
        rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&PcdToOctomapNode::onMapCloud, this, std::placeholders::_1));
      RCLCPP_INFO(
        get_logger(),
        "Subscribing 3D map as PointCloud2 on %s (PCD file not used at startup).",
        map_cloud_topic_.c_str());
    } else if (!pcd_file.empty()) {
      loadPcd(pcd_file);
    } else {
      RCLCPP_INFO(get_logger(), "No map_cloud_topic or pcd_file. Waiting for /pcd_file_cmd.");
    }
  }

private:
  struct Key
  {
    unsigned int k[3];

    bool operator==(const Key & other) const
    {
      return k[0] == other.k[0] && k[1] == other.k[1] && k[2] == other.k[2];
    }
  };

  struct KeyHash
  {
    std::size_t operator()(const Key & key) const
    {
      std::size_t seed = std::hash<unsigned int>{}(key.k[0]);
      seed ^= std::hash<unsigned int>{}(key.k[1]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<unsigned int>{}(key.k[2]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  void onPcdFileCmd(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data.empty()) {
      return;
    }
    loadPcd(msg->data);
  }

  void onMapCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    open3d::geometry::PointCloud o3d;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
      for (; ix != ix.end(); ++ix, ++iy, ++iz) {
        const float x = *ix;
        const float y = *iy;
        const float z = *iz;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }
        o3d.points_.emplace_back(x, y, z);
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_ERROR(
        get_logger(), "PointCloud2 missing x/y/z fields: %s", ex.what());
      return;
    }
    if (o3d.points_.empty()) {
      RCLCPP_WARN(get_logger(), "map_cloud_topic message had no finite points.");
      return;
    }
    fillOctreeFromOpen3d(std::move(o3d), std::string("topic:") + map_cloud_topic_);
  }

  void loadPcd(const std::string & pcd_file)
  {
    open3d::geometry::PointCloud point_cloud;
    if (!open3d::io::ReadPointCloud(pcd_file, point_cloud)) {
      RCLCPP_ERROR(get_logger(), "Failed to read PCD file: %s", pcd_file.c_str());
      return;
    }
    fillOctreeFromOpen3d(std::move(point_cloud), pcd_file);
  }

  void fillOctreeFromOpen3d(open3d::geometry::PointCloud point_cloud, const std::string & source_tag)
  {
    const double voxel_downsample = get_parameter("voxel_downsample_m").as_double();
    if (voxel_downsample > 0.0) {
      point_cloud = *point_cloud.VoxelDownSample(voxel_downsample);
    }

    const std::size_t source_points = point_cloud.points_.size();

    tree_ = std::make_shared<octomap::OcTree>(get_parameter("resolution").as_double());
    const int min_points_per_voxel =
      std::max(1, static_cast<int>(get_parameter("min_points_per_voxel").as_int()));
    const int min_cluster_voxels =
      std::max(1, static_cast<int>(get_parameter("min_cluster_voxels").as_int()));

    std::unordered_map<Key, std::size_t, KeyHash> voxel_counts;
    voxel_counts.reserve(point_cloud.points_.size());
    for (const auto & point : point_cloud.points_) {
      octomap::OcTreeKey raw_key;
      if (!tree_->coordToKeyChecked(
          static_cast<float>(point.x()),
          static_cast<float>(point.y()),
          static_cast<float>(point.z()),
          raw_key))
      {
        continue;
      }
      const Key key{{raw_key.k[0], raw_key.k[1], raw_key.k[2]}};
      ++voxel_counts[key];
    }

    std::unordered_set<Key, KeyHash> occupied_keys;
    occupied_keys.reserve(voxel_counts.size());
    for (const auto & entry : voxel_counts) {
      if (static_cast<int>(entry.second) >= min_points_per_voxel) {
        occupied_keys.insert(entry.first);
      }
    }

    std::size_t removed_cluster_voxels = 0;
    if (min_cluster_voxels > 1 && !occupied_keys.empty()) {
      std::unordered_set<Key, KeyHash> filtered_keys;
      filtered_keys.reserve(occupied_keys.size());
      std::unordered_set<Key, KeyHash> visited;
      visited.reserve(occupied_keys.size());

      for (const auto & seed : occupied_keys) {
        if (visited.find(seed) != visited.end()) {
          continue;
        }

        std::deque<Key> queue;
        std::vector<Key> cluster;
        queue.push_back(seed);
        visited.insert(seed);

        while (!queue.empty()) {
          const Key current = queue.front();
          queue.pop_front();
          cluster.push_back(current);

          for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
              for (int dz = -1; dz <= 1; ++dz) {
                if (dx == 0 && dy == 0 && dz == 0) {
                  continue;
                }

                const auto nx = static_cast<int64_t>(current.k[0]) + dx;
                const auto ny = static_cast<int64_t>(current.k[1]) + dy;
                const auto nz = static_cast<int64_t>(current.k[2]) + dz;
                if (nx < 0 || ny < 0 || nz < 0) {
                  continue;
                }

                const Key neighbor{{
                  static_cast<unsigned int>(nx),
                  static_cast<unsigned int>(ny),
                  static_cast<unsigned int>(nz)}};
                if (occupied_keys.find(neighbor) == occupied_keys.end() ||
                  visited.find(neighbor) != visited.end())
                {
                  continue;
                }

                visited.insert(neighbor);
                queue.push_back(neighbor);
              }
            }
          }
        }

        if (static_cast<int>(cluster.size()) >= min_cluster_voxels) {
          filtered_keys.insert(cluster.begin(), cluster.end());
        } else {
          removed_cluster_voxels += cluster.size();
        }
      }

      occupied_keys = std::move(filtered_keys);
    }

    std::size_t inserted_count = 0;
    for (const auto & key : occupied_keys) {
      octomap::OcTreeKey octo_key;
      octo_key.k[0] = key.k[0];
      octo_key.k[1] = key.k[1];
      octo_key.k[2] = key.k[2];
      tree_->updateNode(tree_->keyToCoord(octo_key), true);
      ++inserted_count;
    }

    tree_->updateInnerOccupancy();
    last_map_source_ = source_tag;
    publishMap();
    RCLCPP_INFO(
      get_logger(),
      "Built OctoMap from %s: source_points=%zu, counted_voxels=%zu, kept_voxels=%zu, "
      "removed_small_cluster_voxels=%zu, occupied_voxels=%zu, min_points_per_voxel=%d, "
      "min_cluster_voxels=%d",
      source_tag.c_str(), source_points, voxel_counts.size(), inserted_count,
      removed_cluster_voxels, tree_->size(), min_points_per_voxel, min_cluster_voxels);
  }

  void publishMap()
  {
    if (!tree_) {
      return;
    }

    octomap_msgs::msg::Octomap map_msg;
    if (!octomap_msgs::binaryMapToMsg(*tree_, map_msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to convert OcTree to octomap message.");
      return;
    }

    map_msg.header.stamp = now();
    map_msg.header.frame_id = get_parameter("frame_id").as_string();
    octomap_pub_->publish(map_msg);
  }

  std::string last_map_source_;
  std::string map_cloud_topic_;
  std::shared_ptr<octomap::OcTree> tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pcd_file_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
