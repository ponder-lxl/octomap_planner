# octomap_planner

一套基于 ROS 2 Humble 的 3D 导航系统（OctoMap + `jie_path_node` 规划）。
本仓库包含三个 ROS 2 包：

- `map_3d_msgs`：地图包保存、加载、导出等自定义服务接口。
- `octomap_3d`：OctoMap 管理包，负责多种地图格式导入、地图包保存/加载、OctoMap 可视化和编辑。
- `octo_planner`：基于 OctoMap 的 3D 路径规划与导航（`jie_path_node` 规划器 + `d1_controller` 路径跟踪控制器 + 局部安全停障 + 路径碰撞检测 + 重规划触发）。

## 新增功能概览

- 将 Gazebo `.world` / `.sdf` 场景转换为 OctoMap。
- 全局静态避障在线重规划
    > 借鉴nav2的在线重规划，还存在一些问题，后续进行优化
- 路径跟踪控制器（差速/全向），含局部静态停障、路径碰撞监控。

## 演示视频

- Bilibili：[【开源】基于ROS2的3D避障导航]()


## 编译

从 ROS 2 工作区根目录编译：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select map_3d_msgs octomap_3d octo_planner
source install/setup.bash
```

如果源码目录移动过，旧 CMake 缓存可能还指向旧路径，可以清理缓存后重编：

```bash
colcon build --packages-select map_3d_msgs octomap_3d octo_planner --cmake-clean-cache
```

## 地图导入

### 导入 Gazebo World / SDF

加载包内示例 world 时，推荐使用 `world_name`：

```bash
ros2 launch octomap_3d import_gazebo_world.launch.py world_name:=field.world
```

加载外部 world 文件时，使用绝对路径：

```bash
ros2 launch octomap_3d import_gazebo_world.launch.py world_file:=/absolute/path/to/map.world
```

如果同时传入 `world_file` 和 `world_name`，优先使用 `world_file`。

`octomap_3d/worlds/` 目录内提供了示例 world 文件，并会随 `octomap_3d` 包安装到 `share/octomap_3d/worlds/`：

- `2_storey.world`：双层建筑/楼层示例。
- `field.world`：场地示例。
- `garage.world`：车库示例。
- `turtlebot3_world.world`：TurtleBot3 默认世界。


## 三维导航

### 加载已保存地图包导航

```bash
ros2 launch octo_planner octomap_package_nav.launch.py map_package_path:=/path/to/map_package
```

流程：`map_package_manager` 自动加载地图包 → 发布 `/octomap` 与规划图层 → `jie_path_node` 三维路径规划 → `/planned_path` → `d1_controller` 路径跟踪 → `/cmd_vel`。


## 路径跟踪控制器

`d1_controller` 是一个完整的路径跟踪控制器节点，提供：

- **路径跟踪**：订阅 `/planned_path`，通过 TF 跟踪，发布 `/cmd_vel`（支持差速/全向模式）。
- **局部安全停障**（`local_safety_stop`）：基于 body 系 LiDAR 点云，实时检测前方障碍物并减速/停车。
- **路径碰撞监控**（`path_collision_monitor`）：沿 `/planned_path` 前方做圆柱膨胀检测，发现碰撞则触发重规划。
- **路径重规划**（`path_replan_trigger`）：blocked 持续触发 + 激光入队。
- **激光 preblocked 构建**（`lidar_preblocked_builder`）：累积多帧点云构建禁行区域。

主要参数见 `octo_planner/config/nav_params.yaml`，包括控制频率、前瞻距离、P 控制器增益、速度上限、死区、到达判定等。



## 参考

本项目基于 [jie_3d_nav](https://github.com/6-robot/jie_3d_nav.git) 开源仓库开发，在此感谢原作团队的开源贡献。