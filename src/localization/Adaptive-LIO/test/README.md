# 不回放 bag 的验证

在本独立测试空间中，从工作空间根目录运行（不会启动驱动或 bag）：

```bash
bash src/Adaptive-LIO/test/run_core_tests.sh
bash src/Adaptive-LIO/test/run_registration_tests.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --symlink-install --packages-up-to adaptive_lio --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/adaptive_lio -R '^adaptive_lio_(core|geometry|registration|pipeline|imu_quality)$' --output-on-failure -V
```

| 测试 | 覆盖 |
|---|---|
| core | 快照、传播、坐标投影、双层重放、超时/历史、主副雷达裁剪、后验注册 |
| registration | 非单位外参、四类轨迹、帧首帧末/区间内/区间外、实际Ceres因子局部雅可比/manifold |
| geometry | 固定R旧结果等价、SPD交叉项、坏R原子拒绝、同R门控、SO(3)切空间、平面/走廊/满秩、采样覆盖 |
| pipeline | 实际ProcessMeasurements：种子、接受、支持不足拒绝、优化后门控拒绝、重新接受；地图/关键帧/点云/状态一致；副雷达发布专用 |
| imu_quality | 单轴削顶、NaN、逆序/重复/重置/间隔、默认停机、有限保持与恢复、噪声复原、带质量元数据的延迟重放 |

测试使用抛异常的断言，Release构建也不会因NDEBUG跳过。核心脚本不依赖ROS；registration脚本查找本机Ceres；pipeline需要完整依赖。不要把头文件测试通过当成节点编译通过。

# 用户自行回放

本次工作空间准备仅做编译和确定性测试。需要时可使用 `python3 src/Adaptive-LIO/test/replay_smoke.py --help`，它只启动被测 LIO、bag 和监视器，默认独立 ROS 域 187/localhost。根目录 `start_bag_odometry.sh`、`play_bag_163.sh` 已适配本测试空间，完整操作见 [工作空间 README](../../../README.md)。

自适应 R 和 IMU 保持未经实车标定，默认关闭。用户自行回放时，可先用固定 R 配置测试静止、平移、旋转、走廊、遮挡恢复；检查拒绝率、恢复、墙厚、漂移及耗时，带真值时再比较精度。不要直接把无真值的连贯轨迹当作精度提升。
