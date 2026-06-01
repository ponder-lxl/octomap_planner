#include "world_sdf_loader.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <sstream>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <octomap/OcTree.h>
#include <tinyxml2.h>

namespace
{
struct Transform3
{
  Eigen::Matrix3d R{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
};

Transform3 compose(const Transform3 & a, const Transform3 & b)
{
  Transform3 out;
  out.R = a.R * b.R;
  out.t = a.R * b.t + a.t;
  return out;
}

Eigen::Vector3d apply(const Transform3 & tf, const Eigen::Vector3d & p)
{
  return tf.R * p + tf.t;
}

std::vector<double> parseDoubles(const std::string & s)
{
  std::istringstream iss(s);
  std::vector<double> vals;
  double v = 0.0;
  while (iss >> v) {
    vals.push_back(v);
  }
  return vals;
}

Transform3 parsePoseElement(const tinyxml2::XMLElement * pose_elem)
{
  Transform3 tf;
  if (!pose_elem || !pose_elem->GetText()) {
    return tf;
  }
  const auto vals = parseDoubles(pose_elem->GetText());
  if (vals.size() < 6) {
    return tf;
  }
  const double roll = vals[3];
  const double pitch = vals[4];
  const double yaw = vals[5];
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  tf.R = (rz * ry * rx).toRotationMatrix();
  tf.t = Eigen::Vector3d(vals[0], vals[1], vals[2]);
  return tf;
}

Transform3 parseAiMatrix(const aiMatrix4x4 & m)
{
  Transform3 tf;
  tf.R << m.a1, m.a2, m.a3, m.b1, m.b2, m.b3, m.c1, m.c2, m.c3;
  tf.t = Eigen::Vector3d(m.a4, m.b4, m.c4);
  return tf;
}

void appendUniquePath(std::vector<std::string> & paths, const std::string & path)
{
  if (path.empty()) {
    return;
  }
  std::error_code ec;
  if (!std::filesystem::exists(std::filesystem::path(path), ec)) {
    return;
  }
  if (std::find(paths.begin(), paths.end(), path) == paths.end()) {
    paths.push_back(path);
  }
}

void appendPackageModels(std::vector<std::string> & paths, const char * package_name)
{
  try {
    const std::string share = ament_index_cpp::get_package_share_directory(package_name);
    appendUniquePath(paths, share + "/models");
  } catch (const std::exception &) {
  }
}
}  // namespace

struct WorldSdfVoxelizer::Impl
{
  explicit Impl(WorldSdfVoxelizer::Options options_in)
  : options(std::move(options_in))
  {
    half_xy_extent_m = 0.5 * options.xy_window_size_m;
    discoverModelPaths();
    resetTree();
  }

  WorldSdfVoxelizer::Options options;
  std::shared_ptr<octomap::OcTree> tree;
  std::vector<std::string> model_paths;
  int shape_count{0};
  double half_xy_extent_m{5.5};

  void resetTree()
  {
    tree = std::make_shared<octomap::OcTree>(options.resolution);
    shape_count = 0;
  }

  void discoverModelPaths()
  {
    model_paths.clear();

    if (const char * env = std::getenv("GAZEBO_MODEL_PATH")) {
      std::stringstream ss(env);
      std::string item;
      while (std::getline(ss, item, ':')) {
        appendUniquePath(model_paths, item);
      }
    }
    if (const char * env = std::getenv("IGN_GAZEBO_RESOURCE_PATH")) {
      std::stringstream ss(env);
      std::string item;
      while (std::getline(ss, item, ':')) {
        appendUniquePath(model_paths, item);
      }
    }

    if (const char * home = std::getenv("HOME")) {
      appendUniquePath(model_paths, std::string(home) + "/.gazebo/models");
    }

    appendUniquePath(model_paths, "/usr/share/gazebo-11/models");
    appendUniquePath(model_paths, "/usr/share/gazebo/models");
    appendPackageModels(model_paths, "turtlebot3_gazebo");
    try {
      const std::string tb3_share =
        ament_index_cpp::get_package_share_directory("turtlebot3_gazebo");
      appendUniquePath(model_paths, tb3_share + "/mesh");
    } catch (const std::exception &) {
    }
    appendPackageModels(model_paths, "gazebo_ros");
    appendPackageModels(model_paths, "gazebo_plugins");

    for (const auto & path : options.extra_model_paths) {
      appendUniquePath(model_paths, path);
    }
  }

  std::string resolveModelUri(const std::string & uri) const
  {
    if (uri.empty()) {
      throw std::runtime_error("empty model uri");
    }

    if (uri.rfind("file://", 0) == 0) {
      return uri.substr(7);
    }
    if (uri.rfind("model://", 0) != 0) {
      std::error_code ec;
      if (std::filesystem::exists(std::filesystem::path(uri), ec)) {
        return uri;
      }
      throw std::runtime_error("unsupported uri: " + uri);
    }

    const std::string rest = uri.substr(8);
    const auto slash = rest.find('/');
    const std::string model_name = slash == std::string::npos ? rest : rest.substr(0, slash);
    const std::string subpath = slash == std::string::npos ? "" : rest.substr(slash + 1);

    for (const auto & base : model_paths) {
      const std::filesystem::path model_dir = std::filesystem::path(base) / model_name;
      std::error_code ec;
      if (!std::filesystem::exists(model_dir, ec)) {
        continue;
      }

      if (!subpath.empty()) {
        const std::filesystem::path full = model_dir / subpath;
        if (std::filesystem::exists(full, ec)) {
          return full.string();
        }
        continue;
      }

      const std::filesystem::path model_sdf = model_dir / "model.sdf";
      if (std::filesystem::exists(model_sdf, ec)) {
        return model_sdf.string();
      }

      const std::filesystem::path model_config = model_dir / "model.config";
      if (std::filesystem::exists(model_config, ec)) {
        tinyxml2::XMLDocument cfg;
        if (cfg.LoadFile(model_config.string().c_str()) == tinyxml2::XML_SUCCESS) {
          const auto * model_elem = cfg.FirstChildElement("model");
          const auto * sdf_elem = model_elem ? model_elem->FirstChildElement("sdf") : nullptr;
          if (sdf_elem && sdf_elem->GetText()) {
            const std::filesystem::path nested = model_dir / sdf_elem->GetText();
            if (std::filesystem::exists(nested, ec)) {
              return nested.string();
            }
          }
        }
      }
    }

    throw std::runtime_error("failed to resolve model uri: " + uri);
  }

  void loadWorldFile(const std::string & world_file)
  {
    resetTree();
    loadSdfFile(world_file, buildCorrectionTransform());
    tree->updateInnerOccupancy();
  }

  Transform3 buildCorrectionTransform() const
  {
    Transform3 tf;
    const double roll = options.world_correction_roll;
    const double pitch = options.world_correction_pitch;
    const double yaw = options.world_correction_yaw;
    if (roll == 0.0 && pitch == 0.0 && yaw == 0.0) {
      return tf;
    }
    const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
    tf.R = (rz * ry * rx).toRotationMatrix();
    return tf;
  }

  void loadSdfFile(const std::string & path, const Transform3 & parent_tf)
  {
    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
      throw std::runtime_error("failed to load sdf file: " + path);
    }

    const tinyxml2::XMLElement * sdf = doc.FirstChildElement("sdf");
    if (!sdf) {
      throw std::runtime_error("no <sdf> root in file: " + path);
    }

    if (const tinyxml2::XMLElement * world = sdf->FirstChildElement("world")) {
      parseWorldElement(world, parent_tf);
      return;
    }

    if (const tinyxml2::XMLElement * model = sdf->FirstChildElement("model")) {
      parseModelElement(model, parent_tf);
      return;
    }

    for (const tinyxml2::XMLElement * model = sdf->FirstChildElement("model");
      model; model = model->NextSiblingElement("model"))
    {
      parseModelElement(model, parent_tf);
    }
  }

  void parseWorldElement(const tinyxml2::XMLElement * world, const Transform3 & parent_tf)
  {
    for (const tinyxml2::XMLElement * child = world->FirstChildElement(); child;
      child = child->NextSiblingElement())
    {
      const std::string tag = child->Name();
      if (tag == "model") {
        parseModelElement(child, parent_tf);
      } else if (tag == "include") {
        parseIncludeElement(child, parent_tf);
      }
    }
  }

  void parseIncludeElement(const tinyxml2::XMLElement * include, const Transform3 & parent_tf)
  {
    const auto * uri_elem = include->FirstChildElement("uri");
    if (!uri_elem || !uri_elem->GetText()) {
      return;
    }
    const Transform3 inc_tf = compose(parent_tf, parsePoseElement(include->FirstChildElement("pose")));
    const std::string resolved = resolveModelUri(uri_elem->GetText());
    loadSdfFile(resolved, inc_tf);
  }

  void parseModelElement(const tinyxml2::XMLElement * model, const Transform3 & parent_tf)
  {
    const Transform3 model_tf = compose(parent_tf, parsePoseElement(model->FirstChildElement("pose")));

    for (const tinyxml2::XMLElement * include = model->FirstChildElement("include"); include;
      include = include->NextSiblingElement("include"))
    {
      parseIncludeElement(include, model_tf);
    }

    for (const tinyxml2::XMLElement * link = model->FirstChildElement("link"); link;
      link = link->NextSiblingElement("link"))
    {
      parseLinkElement(link, model_tf);
    }
  }

  void parseLinkElement(const tinyxml2::XMLElement * link, const Transform3 & model_tf)
  {
    const Transform3 link_tf = compose(model_tf, parsePoseElement(link->FirstChildElement("pose")));
    for (const tinyxml2::XMLElement * collision = link->FirstChildElement("collision"); collision;
      collision = collision->NextSiblingElement("collision"))
    {
      const Transform3 col_tf = compose(link_tf, parsePoseElement(collision->FirstChildElement("pose")));
      const tinyxml2::XMLElement * geom = collision->FirstChildElement("geometry");
      if (!geom) {
        continue;
      }
      parseCollisionGeometry(geom, col_tf);
    }
  }

  void parseCollisionGeometry(const tinyxml2::XMLElement * geom, const Transform3 & col_tf)
  {
    if (const auto * box = geom->FirstChildElement("box")) {
      fillBox(col_tf, box);
      ++shape_count;
    } else if (const auto * cyl = geom->FirstChildElement("cylinder")) {
      fillCylinder(col_tf, cyl);
      ++shape_count;
    } else if (const auto * sph = geom->FirstChildElement("sphere")) {
      fillSphere(col_tf, sph);
      ++shape_count;
    } else if (const auto * plane = geom->FirstChildElement("plane")) {
      fillPlane(col_tf, plane);
      ++shape_count;
    } else if (const auto * mesh = geom->FirstChildElement("mesh")) {
      fillMesh(col_tf, mesh);
      ++shape_count;
    }
  }

  void markPoint(double x, double y, double z)
  {
    if (std::abs(x) > half_xy_extent_m || std::abs(y) > half_xy_extent_m) {
      return;
    }
    octomap::OcTreeKey key;
    const octomap::point3d q(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z));
    if (!tree->coordToKeyChecked(q, key)) {
      return;
    }
    const octomap::point3d center = tree->keyToCoord(key);
    tree->updateNode(center, true);
  }

  void fillBox(const Transform3 & tf, const tinyxml2::XMLElement * box)
  {
    const auto * size_elem = box->FirstChildElement("size");
    if (!size_elem || !size_elem->GetText()) {
      return;
    }
    const auto vals = parseDoubles(size_elem->GetText());
    if (vals.size() < 3) {
      return;
    }
    const double sx = vals[0];
    const double sy = vals[1];
    const double sz = vals[2];
    const double r = tree->getResolution();
    const double min_xy = std::min(sx, sy);
    const double max_xy = std::max(sx, sy);

    const Eigen::Vector3d local_z_in_world = tf.R * Eigen::Vector3d::UnitZ();
    const bool near_horizontal = std::abs(local_z_in_world.z()) > 0.9;
    const bool thin_ground_like = near_horizontal && (sz <= options.ground_surface_max_thickness_m);
    const bool stair_step_like =
      options.enable_stair_step_surface_mode && near_horizontal &&
      (sz <= options.stair_step_max_height_m) &&
      (min_xy <= options.stair_step_max_depth_m) &&
      (max_xy >= options.stair_step_min_width_m);
    if (thin_ground_like || stair_step_like) {
      const double step = std::max(r * 0.5, 1e-3);
      const double sx_eff = std::min(sx, 2.0 * half_xy_extent_m);
      const double sy_eff = std::min(sy, 2.0 * half_xy_extent_m);
      const int nx = std::max(1, static_cast<int>(std::ceil(sx_eff / step)));
      const int ny = std::max(1, static_cast<int>(std::ceil(sy_eff / step)));
      for (int ix = 0; ix <= nx; ++ix) {
        const double x = -sx_eff * 0.5 + (sx_eff * static_cast<double>(ix) / static_cast<double>(nx));
        for (int iy = 0; iy <= ny; ++iy) {
          const double y = -sy_eff * 0.5 + (sy_eff * static_cast<double>(iy) / static_cast<double>(ny));
          const Eigen::Vector3d wp = apply(tf, Eigen::Vector3d(x, y, sz * 0.5));
          markPoint(wp.x(), wp.y(), wp.z());
        }
      }
      return;
    }

    const double min_dim = std::min({sx, sy, sz});
    const double step = (min_dim <= 4.0 * r) ? std::max(r * 0.5, 1e-3) : r;
    const int nx = std::max(1, static_cast<int>(std::ceil(sx / step)));
    const int ny = std::max(1, static_cast<int>(std::ceil(sy / step)));
    const int nz = std::max(1, static_cast<int>(std::ceil(sz / step)));

    for (int ix = 0; ix <= nx; ++ix) {
      const double x = -sx * 0.5 + (sx * static_cast<double>(ix) / static_cast<double>(nx));
      for (int iy = 0; iy <= ny; ++iy) {
        const double y = -sy * 0.5 + (sy * static_cast<double>(iy) / static_cast<double>(ny));
        for (int iz = 0; iz <= nz; ++iz) {
          const double z = -sz * 0.5 + (sz * static_cast<double>(iz) / static_cast<double>(nz));
          const Eigen::Vector3d wp = apply(tf, Eigen::Vector3d(x, y, z));
          markPoint(wp.x(), wp.y(), wp.z());
        }
      }
    }
  }

  void fillCylinder(const Transform3 & tf, const tinyxml2::XMLElement * cyl)
  {
    const auto * r_elem = cyl->FirstChildElement("radius");
    const auto * l_elem = cyl->FirstChildElement("length");
    if (!r_elem || !l_elem || !r_elem->GetText() || !l_elem->GetText()) {
      return;
    }
    const double radius = std::stod(r_elem->GetText());
    const double length = std::stod(l_elem->GetText());
    const double res = tree->getResolution();
    for (double x = -radius; x <= radius; x += res) {
      for (double y = -radius; y <= radius; y += res) {
        if (x * x + y * y > radius * radius) {
          continue;
        }
        for (double z = -length * 0.5; z <= length * 0.5; z += res) {
          const Eigen::Vector3d wp = apply(tf, Eigen::Vector3d(x, y, z));
          markPoint(wp.x(), wp.y(), wp.z());
        }
      }
    }
  }

  void fillSphere(const Transform3 & tf, const tinyxml2::XMLElement * sph)
  {
    const auto * r_elem = sph->FirstChildElement("radius");
    if (!r_elem || !r_elem->GetText()) {
      return;
    }
    const double radius = std::stod(r_elem->GetText());
    const double res = tree->getResolution();
    for (double x = -radius; x <= radius; x += res) {
      for (double y = -radius; y <= radius; y += res) {
        for (double z = -radius; z <= radius; z += res) {
          if (x * x + y * y + z * z > radius * radius) {
            continue;
          }
          const Eigen::Vector3d wp = apply(tf, Eigen::Vector3d(x, y, z));
          markPoint(wp.x(), wp.y(), wp.z());
        }
      }
    }
  }

  void fillPlane(const Transform3 & tf, const tinyxml2::XMLElement * plane)
  {
    const auto * size_elem = plane->FirstChildElement("size");
    if (!size_elem || !size_elem->GetText()) {
      return;
    }
    const auto vals = parseDoubles(size_elem->GetText());
    if (vals.size() < 2) {
      return;
    }
    const double sx = vals[0];
    const double sy = vals[1];
    const double r = tree->getResolution();
    const double step = std::max(r * 0.5, 1e-3);
    const double sx_eff = std::min(sx, 2.0 * half_xy_extent_m);
    const double sy_eff = std::min(sy, 2.0 * half_xy_extent_m);
    const int nx = std::max(1, static_cast<int>(std::ceil(sx_eff / step)));
    const int ny = std::max(1, static_cast<int>(std::ceil(sy_eff / step)));
    for (int ix = 0; ix <= nx; ++ix) {
      const double x = -sx_eff * 0.5 + (sx_eff * static_cast<double>(ix) / static_cast<double>(nx));
      for (int iy = 0; iy <= ny; ++iy) {
        const double y = -sy_eff * 0.5 + (sy_eff * static_cast<double>(iy) / static_cast<double>(ny));
        const Eigen::Vector3d wp = apply(tf, Eigen::Vector3d(x, y, 0.0));
        markPoint(wp.x(), wp.y(), wp.z());
      }
    }
  }

  void sampleTriangle(
    const Transform3 & tf,
    double v0x, double v0y, double v0z,
    double v1x, double v1y, double v1z,
    double v2x, double v2y, double v2z)
  {
    const Eigen::Vector3d v0(v0x, v0y, v0z);
    const Eigen::Vector3d v1(v1x, v1y, v1z);
    const Eigen::Vector3d v2(v2x, v2y, v2z);
    const double step = std::max(tree->getResolution() * 0.5, 1e-3);
    const double max_edge = std::max({(v1 - v0).norm(), (v2 - v1).norm(), (v0 - v2).norm()});
    const int steps = std::max(1, static_cast<int>(std::ceil(max_edge / step)));

    for (int i = 0; i <= steps; ++i) {
      for (int j = 0; j <= steps - i; ++j) {
        const double u = static_cast<double>(i) / static_cast<double>(steps);
        const double v = static_cast<double>(j) / static_cast<double>(steps);
        if (u + v > 1.0 + 1e-9) {
          continue;
        }
        const Eigen::Vector3d local = (1.0 - u - v) * v0 + u * v1 + v * v2;
        const Eigen::Vector3d wp = apply(tf, local);
        markPoint(wp.x(), wp.y(), wp.z());
      }
    }
  }

  void fillMesh(const Transform3 & tf, const tinyxml2::XMLElement * mesh)
  {
    const auto * uri_elem = mesh->FirstChildElement("uri");
    if (!uri_elem || !uri_elem->GetText()) {
      return;
    }

    Eigen::Vector3d scale(1.0, 1.0, 1.0);
    if (const auto * scale_elem = mesh->FirstChildElement("scale")) {
      if (scale_elem->GetText()) {
        const auto vals = parseDoubles(scale_elem->GetText());
        if (vals.size() >= 3) {
          scale = Eigen::Vector3d(vals[0], vals[1], vals[2]);
        } else if (!vals.empty()) {
          scale = Eigen::Vector3d(vals[0], vals[0], vals[0]);
        }
      }
    }

    const std::string mesh_path = resolveModelUri(uri_elem->GetText());
    Assimp::Importer importer;
    const aiScene * scene = importer.ReadFile(
      mesh_path,
      aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenNormals);
    if (!scene || !scene->mRootNode) {
      throw std::runtime_error(
        std::string("assimp failed to load mesh: ") + mesh_path + " (" + importer.GetErrorString() +
        ")");
    }

    std::function<void(const aiNode *, const Transform3 &)> process_node;
    process_node = [&](const aiNode * node, const Transform3 & parent_tf) {
      const Transform3 node_tf = compose(parent_tf, parseAiMatrix(node->mTransformation));
      for (unsigned mi = 0; mi < node->mNumMeshes; ++mi) {
        const aiMesh * ai_mesh = scene->mMeshes[node->mMeshes[mi]];
        for (unsigned fi = 0; fi < ai_mesh->mNumFaces; ++fi) {
          const aiFace & face = ai_mesh->mFaces[fi];
          if (face.mNumIndices != 3) {
            continue;
          }
          const aiVector3D & p0 = ai_mesh->mVertices[face.mIndices[0]];
          const aiVector3D & p1 = ai_mesh->mVertices[face.mIndices[1]];
          const aiVector3D & p2 = ai_mesh->mVertices[face.mIndices[2]];
          const Eigen::Vector3d v0(p0.x * scale.x(), p0.y * scale.y(), p0.z * scale.z());
          const Eigen::Vector3d v1(p1.x * scale.x(), p1.y * scale.y(), p1.z * scale.z());
          const Eigen::Vector3d v2(p2.x * scale.x(), p2.y * scale.y(), p2.z * scale.z());
          sampleTriangle(
            node_tf, v0.x(), v0.y(), v0.z(), v1.x(), v1.y(), v1.z(), v2.x(), v2.y(), v2.z());
        }
      }
      for (unsigned ci = 0; ci < node->mNumChildren; ++ci) {
        process_node(node->mChildren[ci], node_tf);
      }
    };

    process_node(scene->mRootNode, tf);
  }
};

WorldSdfVoxelizer::WorldSdfVoxelizer(Options options)
: impl_(std::make_unique<Impl>(std::move(options)))
{
}

WorldSdfVoxelizer::~WorldSdfVoxelizer() = default;

void WorldSdfVoxelizer::loadWorldFile(const std::string & world_file)
{
  impl_->loadWorldFile(world_file);
}

std::shared_ptr<octomap::OcTree> WorldSdfVoxelizer::tree() const
{
  return impl_->tree;
}

int WorldSdfVoxelizer::shapeCount() const
{
  return impl_->shape_count;
}
