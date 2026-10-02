# rmu_gazebo_simulator

## 简介

rmu_gazebo_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境，为 RoboMaster University 中的机器人算法开发提供仿真环境，方便测试 AI 算法，加快开发效率。

目前 rmu_gazebo_simulator 提供以下功能：

- rmul_2024、rmuc_2024、rmuc_2025 仿真世界模型

- 网页端局域网联机对战

- 机器人底盘、云台、射击控制

| rmul_2024 | rmuc_2024 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_fly.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_fly_image.1aoyoashvj.gif)|

## 一. 基本使用

### 1.1 环境配置

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)

```sh
cd ~/ros_ws/src

git clone https://github.com/gezp/sdformat_tools.git
git clone https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_interfaces.git
git clone https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_core.git
git clone https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gazebo.git
git clone https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_gz_resources.git --depth=1
git clone https://github.com/SMBU-PolarBear-Robotics-Team/rmu_gazebo_simulator.git
git clone https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_robot_description.git

pip install xmacro
```

```sh
cd ..
colcon build --symlink-install
```

### 1.2 启动仿真环境

- 启动仿真环境

    ```sh
    ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
    ```

    **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

- 控制机器人移动

    ```sh
    ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
    #根据提示进行输入，支持平移与自旋
    ```

- 键盘控制：

  - 机器人云台

    ```sh
    ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入，支持绝对角度控制
    ```

  - 机器人射击

    ```sh
    ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入
    ```

- 网页端控制

    支持局域网内联机操作，只需要将 localhost 改为主机 ip 即可。

  - 操作手端

    <http://localhost:5000/>

    ```sh
    python3 src/rmu_gazebo_simulator/scripts/player_web/main_no_vision.py
    ```

  - 裁判系统端

    <http://localhost:2350/>

    ```sh
    python3 src/rmu_gazebo_simulator/scripts/referee_web/main.py
    ```

### 1.3 切换仿真世界

修改 [gz_world.yaml](./config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`, `rmul_2025`, `rmuc_2025`, `rmuc_2026`

### RMUC 2026 场景接入

当前默认 `world: "rmuc_2026"`。`robots.rmuc_2026` 通过 YAML 别名复用
`robots.rmuc_2025`：仍只生成 `red_standard_robot1`，初始位置为
`(3.4, 9.5, 0.88)`，yaw 为 `0.0`。修改 rmuc25 的机器人配置也会同步到 rmuc26。

加载链：`sim.sh` → 安装目录中的 `bringup_sim.launch.py` → `config/gz_world.yaml`
→ `gazebo.launch.py` → `resource/worlds/rmuc_2026_world.sdf`
→ `resource/models/{rmuc_2026,speed_bump}`。`spawn_robots.launch.py` 独立读取同一 YAML。
启动脚本、机器人模型、桥接、控制器、裁判配置均沿用现有配置；world 名仍为 `default`，
保留 `/world/default/...` 服务路径及本仓库 Gazebo 8 插件、灯光设置。

场景模型和两个减速带来自 [git-alphabet/simulation](https://github.com/git-alphabet/simulation/tree/11561067834255e7b15bf0be89de2deaa15e9c07/src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource)，
固定提交 `11561067834255e7b15bf0be89de2deaa15e9c07`；模型文件及网格原样保留。
上游场地以中心附近为原点，为保持 rmuc25 的坐标基准，对场地与两个减速带统一平移
`(14.5, 7.375655746, -0.158656120)` 米：场地中心对齐 `(14.5, 8)`，地板底面为
`z=0`，出生点处地板表面为 `z=0.2`。保留上游场地缩放 `0.1` 和减速带缩放 `0.001`。
此变换只作用于场景，不修改机器人初始位姿。新赛季障碍布局仍与 2025 不同。

2026 world 另有四面灰色静态围墙，沿场地外缘闭合布置，厚 `0.055 m`，
底部 `z=0.059461`、顶部 `z=18.754619`，与 rmuc25 网格高墙高度一致。
墙体同时包含 visual 和 collision，可用于渲染、激光观测与物理碰撞。

新增资源后需重新安装仿真包（CMake 已自动安装整个 resource 目录）：

```sh
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rmu_gazebo_simulator
source install/setup.bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

若需回到旧场地，只需把 `world` 改回 `"rmuc_2025"`。

## 配套导航仿真仓库

- 2025 SMBU PolarBear Sentry Navigation

    [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav.git)

    ![cmu_nav_v1_0](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

rmu_gazebo_simulator is provided under Apache License 2.0.
