# 实车集成

算法、构建配置和测试来自仓库根目录 `Adaptive-LIO/src/Adaptive-LIO`。
导航仍通过 `ros2 launch adaptive_lio run.launch.py` 启动当前包；
`bringup/startup.launch.py` 已使用此入口，无需更换启动命令。
根目录副本添加了 `COLCON_IGNORE`，避免重复发现 adaptive_lio 和 livox_ros_driver2。
现有 Livox 驱动及其网络、安装补偿配置继续使用 `src/lidar/livox_ros_driver2`。

## 参数迁移

`config/mapping_m.yaml` 保留当前实车配置：

- 主雷达和 IMU：192.168.1.173；副雷达话题：192.168.1.160。
- `common.gnorm: 9.805`：现有 Livox 驱动直接发布 g 单位加速度，里程计入口转换为 m/s²；静止时转换后的模长应约为 9.8。
- `aft_mapped -> base_link`：`[-0.04, 0.165, -0.33, 0, 0, 0]`。
- 主雷达外参、有效体素地图参数（`odometry.size_voxel_map: 0.5`）、点云高度与降采样、车体包围盒保持现有值。
- 建图保持启用，地图保存路径为 `./map_scantext07093.pcd`。
- 副雷达保持仅用于点云输出，未启用配准融合；残余外参为单位变换，没有引入另一台车的标定。

新版本采用 `IMU_DESKEW_END`，移除了已废弃的 CT-ICP 参数和旧饱和阈值。
旧 `map_options` 和 `preprocess.degenerate_window_num` 会被新版拒绝，已移除；地图使用 `odometry` 下的体素配置。
IMU 饱和量程暂为零（关闭对应检测），待确认传感器实际 SI 量程后填写 `imu_quality`。
TF 使用真实 IMU 状态时间，新版不支持 `fusion_restamp_tf_to_now`。

## 编译与验证

```sh
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to adaptive_lio --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select adaptive_lio --ctest-args -R '^adaptive_lio_'
colcon test-result --verbose
source install/setup.bash
ros2 launch bringup startup.launch.py
```

此替换针对导航已使用的 Adaptive-LIO。`bringup/mapping.launch.py` 仍是原有 Point-LIO 建图流程。
上线前应实测 IMU 量纲、TF 连通性、里程计连续性和点云位置；本地编译与单元测试不能替代实车验证。

2026-10-16：ROS Jazzy 下主程序及测试编译成功，`adaptive_lio_core`、
`adaptive_lio_geometry`、`adaptive_lio_registration`、`adaptive_lio_pipeline`、
`adaptive_lio_imu_quality` 五项测试全部通过；流水线测试使用本包实车参数文件。

## 点云链路排查（2026-10-17）

`/home/sb/zstart.sh` 使用本工作空间的 `bringup/startup.launch.py`。
点云链路为 `/livox/lidar_192_168_1_173` → Adaptive-LIO → `/livox/scan` →
CloudAccumulator → `/lidar/accumulated_cloud` → Patchwork++ → `/nonground`。
导航 RViz 默认显示 `/nonground`，原始 `/livox/scan` 显示项默认关闭。

上次运行日志明确记录 `/livox/scan` 的 RELIABILITY_QOS_POLICY 不兼容：
Adaptive-LIO 使用 Best Effort，而 terrain_analysis 原来请求 Reliable。
terrain_analysis 已改为 SensorDataQoS，与当前点云发布端兼容。

实车隔离验证复现了直接断点：173 雷达启动时 IMU 存在约 0.09～1.77 秒的间隔，
质量门限在初始化前就将两条估计链永久置为故障，因而无 `/odom` 和 `/livox/scan`。
入口现等待一秒连续有效 IMU（相邻间隔不超过 50 ms），再开始原有初始化。
此等待只作用于启动阶段；开始接收后，原有时间缺口、非有限值和饱和保护继续生效。

检查驱动的 `comm/pub_handler.cpp` 和 `lddc.cpp` 后确认：加速度只做旋转，未进行 g 到 m/s² 的换算。
因此将此前沿用的 `gnorm: 1` 修正为 `9.805`，保留雷达 IP、安装外参和车体滤波设置。

旧的 2026-08-03 录包存在超过 50 ms 的 IMU 时间缺口，回放会触发新版停止保护；
这与 QoS 问题独立，未放宽该保护。连续的 rosbag2_gym 用于进一步验证输出。

修复后已重新编译 adaptive_lio 和 terrain_analysis，五项核心测试全部通过（包含新增启动时序回归）。
在隔离 ROS 域 189 仅启动实车雷达、里程计、Patchwork++ 和 terrain_analysis，18 秒观测结果：

- 主雷达点云 150 帧，原始 IMU 2589 条，实测加速度模长均值 0.9976 g。
- `/livox/scan` 142 帧，全部非空，每帧 4905～5363 点；`/odom` 2567 条。
- `/nonground` 142 帧，其中 141 帧非空；首帧因 TF 缓存尚未覆盖而为空。
- `/terrain_map` 142 帧，全部非空；运行期间未触发 IMU 故障。

诊断结束后这些节点均已停止。重新运行 zstart 即可使用已安装的新程序。
此验证未启动导航、串口或底盘控制，也未打开 RViz；实际显示仍由导航的 map → odom 定位链提供。
另观察到里程计在 SIGINT 退出清理阶段出现 DDS 句柄销毁异常；与此次运行期间点云中断独立，尚未修复。

### 完整 zstart 负载下的后续修复

上述短时隔离验证不足以覆盖完整启动：用户随后启动导航、RViz 和视觉时，
里程计在 11:10:13 再次触发 IMU 时间故障，进程存活但两个输出线程均已停止。
只等待启动阶段连续采样无法解决运行中的假时间缺口。

驱动 `comm/pub_handler.cpp` 原先在无 PTP/GPS 同步时，把每次回调的主机执行时刻
当作采样时间。现改为每台雷达独立建立设备时间到主机时间的固定偏移，
同一雷达的 IMU 和点云共用偏移，保留设备原始采样间隔；
已同步的 PTP/GPS 时间直接透传。该偏移不是跨设备硬件时钟同步，也不掩盖设备重启、真实丢包或真实采样缺口。
IMU 故障日志增加当前、上一条时间戳和间隔，便于区分后续问题。

用户重新启动完整 zstart 后，只读连续观测 45 秒：

- `/livox/scan` 收到 447 帧，`/nonground` 收到 422 帧，`/odom` 收到 6030 条。
- 观测结束时点云每帧 7009 点，`/nonground` 每帧 6321 点，仍在持续输出。
- `map → odom → aft_mapped → base_link` 和 `map → base_link_fake` 查询均成功，动态 TF 持续刷新。
- 未新启导航或发送控制命令；验证直接观察用户已启动的完整系统。

驱动的时钟回归测试覆盖主机调度延迟、主机时间跳变、乱序、多设备独立偏移、
同设备点云/IMU 共用时间、真实采样缺口和设备时间复位；真实缺口与复位继续交由里程计故障保护处理。
