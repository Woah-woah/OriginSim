# map_server

独立 ROS 2 地图工程。参考[浙大 Hello World HWSentryNav26 map_server](https://github.com/Polyacetone/HWSentryNav26/tree/main/map_server)的地图集中提供方式，保留现有导航使用的 Nav2 uint8 语义，自行实现网格与几何工具；生产代码不依赖 Nav2。

本阶段对齐原配置实际启用的静态层、footprint 清理、膨胀层、全局地图和局部滚动窗口。没有实现 Nav2 的整套插件、增量更新、激光障碍层或 lifecycle 管理。DynamicMap 和 TerrainMap 后续单独扩展，本阶段不实现。

## 模块

| 文件 | 职责 |
| --- | --- |
| `static_map_loader.cpp` | YAML、PGM/PNG 加载，支持 trinary、scale、raw；输出标准占据地图 |
| `static_map_node.cpp` | 独立发布 `/map`，提供 `map_server/map` GetMap 服务 |
| `costmap_2d.hpp` | 自研行优先 uint8 网格、坐标转换、互斥锁、深拷贝快照 |
| `static_layer.cpp` | 占据值解释与静态地图旋转归一化 |
| `inflation_layer.cpp` | 精确欧氏距离膨胀 |
| `footprint_layer.cpp` | 机器人 footprint 清理 |
| `map_geometry.cpp` / `footprint.hpp` | 机器人尺寸、footprint 解析与碰撞检查 |
| `costmap_node.cpp` | 全局/局部地图生成、TF 检查、窗口更新与发布 |
| `costmap_client.cpp` | executor 跨进程订阅、消息验证与地图失效检测 |

`nav_executor` 不再创建或激活 Nav2 Costmap2DROS。它只保存全局和局部消息快照，并把自研 `Costmap2D` 传给 topology planner 和 MPPI。全局规划线程对整次拓扑规划使用同一份深拷贝；控制器读取局部地图时持有网格锁。

地图客户端启动时允许空网格。MPPI 的 `FollowPath.max_robot_pose_search_dist` 默认 `0` 表示自动选择当前地图最大边长的一半，在处理路径时读取有效地图尺寸；不会把启动时的空网格尺寸固化为搜索范围。显式设置正数时使用指定的米数。

## 消息语义

- `/map`：`nav_msgs/OccupancyGrid`，未知 `-1`，占据概率 `0..100`，可靠且 transient-local。
- `/global_costmap/costmap_raw`、`/local_costmap/costmap_raw`：`map_server/msg/Costmap`，保存原始 uint8 数据，可靠且 transient-local。
- `/global_costmap/costmap`、`/local_costmap/costmap`：供 RViz 使用的 OccupancyGrid 可视化；将代价明确转换到 `-1, 0..100`，不用于恢复原始规划代价。

内部常量在 `cost_values.hpp` 自行定义：`FREE_SPACE=0`、`INSCRIBED_INFLATED_OBSTACLE=253`、`LETHAL_OBSTACLE=254`、`NO_INFORMATION=255`。普通膨胀代价为 `1..252`。静态层默认以占据值 `100` 为 lethal，其他已知值为 free；`trinary_costmap=false` 时按 lethal threshold 缩放代价。未知空间是否保留由 `track_unknown_space` 控制。

Costmap 消息包含 header（frame_id 和时间戳）、MapMetaData（分辨率、宽高、origin、加载时间）、行优先原始数据和 current 状态，还携带机器人与膨胀设置。costmap 是二维轴对齐网格，origin orientation 必须是单位四元数；带 yaw 的输入 `/map` 会先归一化到其 frame 内的轴对齐网格。客户端拒绝无效几何、尺寸不匹配或机器人设置不一致的消息。

可视化转换为：`0→0`、`1..252→1..98`、`253→99`、`254→100`、`255→-1`。转换有损，不能把该视图作为原始代价数据。

## 配置和启动

完整导航使用 `bringup/params/nav2_params.yaml`，其中包含地图发布、global/local costmap、executor 和 planner 参数。launch 直接把参数文件传给节点；executor 的地图客户端读取其中对应的 global/local 设置，避免 footprint、frame 或膨胀参数不一致。参数键使用普通节点名，对应默认的空命名空间。修改参数后重启导航；当前不支持运行时参数更新。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to nav_executor nav bringup
source install/setup.bash

# 终端 1：启动 /map 和两套 costmap，使用导航中的同一份参数。
ros2 launch map_server map_server.launch.py \
  map:=/absolute/path/to/map.yaml use_sim_time:=true \
  map_params_file:=/absolute/path/to/bringup/params/nav2_params.yaml

# 终端 2：启动 executor 和内部 planner/controller。
ros2 launch nav bringup_launch.py use_sim_time:=true \
  params_file:=/absolute/path/to/bringup/params/nav2_params.yaml
```

自定义导航配置传 `params_file:=/absolute/path/to/navigation.yaml`，拓扑地图传 `topology_file:=/absolute/path/to/topo.yaml`。默认机器人 frame 是 `base_link_fake`，全局 frame 是 `map`，局部 frame 是 `odom`。需要有效 TF：`map → odom → base_link_fake`。`use_sim_time=true` 时还需要 `/clock`。

仅运行地图生产端时，可使用 `ros2 launch map_server map_server.launch.py map:=/absolute/path/to/map.yaml`，默认读取 `bringup/params/nav2_params.yaml`，与导航默认使用同一份配置。自定义配置时，地图端的 `map_params_file` 和导航端的 `params_file` 应指定同一份文件。

`bringup/startup.launch.py` 通过 `map_server.launch.py` 启动 `/map` 与两套 costmap，并传入同一份 `nav2_params.yaml`；nav 包只启动 executor，不启动地图节点。原来的导航启动开关仍保持其原有状态。导航参数路径 `bringup/params/nav2_params.yaml` 为兼容现有入口保留名称，但其中已移除 Nav2 costmap 插件配置。

膨胀采用精确欧氏距离和 `252*exp(-cost_scaling_factor*(distance-inscribed_radius))`，半径内侧为 253，障碍为 254。局部窗口先在包含膨胀 halo 的区域投影静态地图，再清理 footprint、膨胀、裁剪，窗口外的障碍也会影响边缘代价。TF 不可用时发布 `current=false`；生产端掉线、消息过期或格式错误时客户端判定地图失效，executor 停止控制输出。

## 本次验证

开发期间运行了临时对照测试：网格坐标与快照、概率/代价转换、旋转地图、随机地图逐格 Nav2 膨胀对照（含未知空间策略）、footprint 碰撞对照、现有 PGM 加载对照和客户端失效处理。另以隔离 ROS 域运行独立进程测试，覆盖 late subscriber、滚动窗口、窗口外障碍膨胀、map/odom 旋转平移、规划与 MPPI 输出，以及地图进程退出后的零速输出。按要求验证后删除临时测试源码和测试 CMake 配置，仅保留实现及构建产物。
