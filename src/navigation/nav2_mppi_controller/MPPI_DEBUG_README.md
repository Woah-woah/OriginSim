# Nav2 MPPI 调试手册（ROS 2 Jazzy / RoboMaster 哨兵）

> 适用场景：ROS 2 Jazzy + Nav2 `nav2_mppi_controller`，Omni 全向底盘，面向 RoboMaster 哨兵高速导航与动态避障调试。  
> 核心目标：先把“预测范围、运动能力、costmap、采样空间”调对，再调 Critic 权重。不要一开始就靠改 `cost_weight` 补偿基础参数错误。

---

## 1. 当前配置示例

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"

  time_steps: 56
  model_dt: 0.06

  batch_size: 2000
  iteration_count: 1

  vx_std: 0.5
  vy_std: 0.5
  wz_std: 0.1

  vx_max: 3.0
  vx_min: -3.0
  vy_max: 3.0
  wz_max: 0.1

  ax_max: 20.0
  ax_min: -20.0
  ay_max: 20.0
  ay_min: -20.0

  prune_distance: 8.5
  transform_tolerance: 0.1

  temperature: 0.3
  gamma: 0.015

  motion_model: "Omni"

  visualize: true
  publish_optimal_trajectory: true

  TrajectoryVisualizer:
    trajectory_step: 5
    time_step: 3

  critics:
    - "ObstaclesCritic"
    - "GoalCritic"
    - "PathAlignCritic"
    - "PathFollowCritic"
    - "TwirlingCritic"
```

---

# 2. 调 MPPI 前先算 4 个量

## 2.1 Controller 周期

```text
controller_period = 1 / controller_frequency
```

Jazzy MPPI 中，最稳妥的做法是：

```text
model_dt = controller_period
```

例如：

```yaml
controller_frequency: 20.0
model_dt: 0.05
```

如果 `controller_period > model_dt`，MPPI 会直接报：

```text
Controller period more then model dt, set it equal to model dt
```

所以每次改 `model_dt` 后都先检查：

```bash
ros2 param get /controller_server controller_frequency
```

---

## 2.2 Prediction Horizon

公式：

```text
prediction_time = time_steps × model_dt
```

当前配置：

```text
56 × 0.06 = 3.36 s
```

所以你的 MPPI 每次向未来预测约 **3.36 秒**。

---

## 2.3 理论最大前向预测距离

粗略估计：

```text
prediction_distance_x ≈ prediction_time × vx_max
```

当前：

```text
3.36 × 3.0 = 10.08 m
```

注意：这是“在整个 horizon 都保持 3 m/s”时的理论上界，不代表实际每次都会看满 10.08 m。

对于 Omni 底盘，如果 `vx`、`vy` 同时较大，平移合速度会更高，因此二维轨迹的实际空间跨度可能进一步增大。

---

## 2.4 PathAlign / PathFollow 的 offset 基线

官方调参思路可以近似写成：

```text
offset_from_furthest
≈ prediction_distance / path_resolution / 3
```

如果：

```text
prediction_distance = 10.08 m
path_resolution = 0.05 m
```

那么：

```text
10.08 / 0.05 / 3 ≈ 67
```

所以当前：

```yaml
offset_from_furthest: 25
```

对应的是更短的有效前视距离。

这不代表 25 一定错误，但要知道它已经不再对应当前 3.36 s、3 m/s 的最大预测尺度。

---

# 3. `vx_max` 和 `vx_std` 完全不是一个东西

## `vx_max`

表示硬速度上限：

```yaml
vx_max: 3.0
vx_min: -3.0
```

表示合法输出范围大致受限于：

```text
-3.0 <= vx <= 3.0 m/s
```

---

## `vx_std`

表示 MPPI 每轮采样时，在当前控制序列附近探索多远。

例如：

```yaml
vx_std: 0.5
```

近似理解为：

```text
vx_sample = vx_current + N(0, 0.5²)
```

如果当前控制中心：

```text
vx_current = 1.0 m/s
```

那么大约：

```text
68% 样本：0.5 ~ 1.5
95% 样本：0.0 ~ 2.0
99.7% 样本：-0.5 ~ 2.5
```

最终还会受到：

- `vx_min / vx_max`
- 加速度限制
- Critic cost
- motion model

等约束。

因此：

```text
vx_max = “允许走到哪里”
vx_std = “每一轮敢在当前解附近搜多远”
```

---

# 4. 为什么当前这套参数跑得很快

当前配置实际上同时做了四件事：

## 4.1 速度上限大

```yaml
vx_max: 3.0
vy_max: 3.0
```

相比 1.5 m/s，上限直接翻倍。

---

## 4.2 平移搜索范围大

```yaml
vx_std: 0.5
vy_std: 0.5
```

比 0.2 更容易在一轮采样中找到明显更高速的解。

---

## 4.3 加速度限制极宽

```yaml
ax_max: 20.0
ax_min: -20.0
ay_max: 20.0
ay_min: -20.0
```

这会让 optimizer 认为底盘能够非常快地改变速度。

优点：

- 起步猛
- 高速响应快
- 更容易追上高速度解

风险：

- 如果实车真实加速度达不到，预测轨迹会和执行轨迹失配
- 高速避障时可能出现“optimizer 认为来得及，实车实际刹不住”

因此最终实车参数最好根据真实底盘加速度重新标定。

---

## 4.4 基本不允许旋转

```yaml
wz_max: 0.1
wz_std: 0.1

TwirlingCritic:
  twirling_cost_weight: 10.0
```

这等于告诉 Omni MPPI：

```text
x / y：大胆平移
yaw：少转
```

所以它会更倾向于：

```text
保持底盘朝向 + 高速横移 / 斜移
```

而不是：

```text
边转车头 + 边走
```

这会让轨迹观感非常“干脆”。

---

# 5. `model_dt`、`time_steps`、`controller_frequency`

优先关系：

```text
controller_frequency
        ↓
controller_period
        ↓
model_dt
        ↓
time_steps
        ↓
prediction horizon
```

推荐步骤：

1. 先确定控制频率
2. 令 `model_dt ≈ 1/controller_frequency`
3. 再根据需要的预测时间选择 `time_steps`

例如希望：

```text
controller_frequency = 20 Hz
prediction horizon ≈ 3 s
```

那么：

```text
model_dt = 0.05
time_steps ≈ 60
```

不要先随便设 `model_dt`，再反过来硬配 controller。

---

# 6. `batch_size` / `iteration_count`

当前：

```yaml
batch_size: 2000
iteration_count: 1
```

这是合理的 MPPI 调试思路。

需要更充分的搜索时：

```text
优先 batch_size ↑
```

而不是：

```text
iteration_count ↑↑↑
```

因为每个控制周期本身就会继承上一轮控制序列继续优化。

需要注意 CPU：

```text
batch_size × time_steps × critics
```

决定了大量计算量。

当前：

```text
2000 × 56 = 112000 个 rollout 时间点 / iteration
```

再乘上多个 Critic。

---

# 7. `temperature`

当前：

```yaml
temperature: 0.3
```

理解为：

> MPPI 对低 cost 轨迹有多“挑”。

较低：

```text
更偏向最低 cost 轨迹
```

较高：

```text
更多候选轨迹共同影响最终控制
```

调车初期通常保持默认附近，不要优先改。

---

# 8. `gamma`

当前：

```yaml
gamma: 0.015
```

属于 MPPI 内部控制能量 / 平滑性相关参数。

建议：

```text
没有明确问题时不要动
```

优先调：

- 速度 / 加速度
- std
- prediction horizon
- costmap
- Obstacles
- PathFollow
- PathAlign

---

# 9. Costmap 和 Prediction Horizon

这一点非常重要。

如果 local costmap 是 centered rolling window：

```yaml
width: 8.4
height: 8.4
```

机器人到边界实际上大约只有：

```text
8.4 / 2 = 4.2 m
```

不是 8.4 m。

而当前最大理论前向预测距离是：

```text
10.08 m
```

如果真的要求“整个最大速度预测轨迹都落在 costmap 内”，那么 10 m 级预测距离需要非常大的 local costmap。

实际工程中通常不会简单粗暴地把 local costmap 扩到 20 m，而是综合调整：

- `time_steps`
- `model_dt`
- `vx_max`
- local costmap 尺寸
- 实际传感器有效距离
- DynamicLayer 有效范围
- CPU 占用

因此不要机械追求：

```text
prediction_distance 越大越好
```

---

# 10. `prune_distance`

作用：

> 控制 MPPI 取机器人前方多少全局路径。

当前：

```yaml
prune_distance: 8.5
```

调参原则：

- 不能小到把 Critic 所需要的前方路径截掉
- 不必盲目等于最大理论预测距离
- 应与 prediction horizon、path resolution、PathFollow / PathAlign 的 offset 一起看

如果 MPPI 高速运行时发现：

- `transformed_global_plan` 太短
- PathAlign 很早失效
- PathFollow 前视不足

再增加它。

---

# 11. GoalCritic

当前：

```yaml
GoalCritic:
  cost_weight: 5.0
  threshold_to_consider: 2.8
```

注意：

```text
threshold_to_consider 的单位是米
```

不是秒。

所以：

```yaml
threshold_to_consider: 2.8
```

表示：

> 距离目标约 2.8 m 内开始重点考虑 GoalCritic。

不是：

> prediction horizon = 2.8 s

---

# 12. PathFollowCritic

作用：

> 让机器人沿任务方向继续取得进展。

可以理解为：

```text
“别只顾避障，继续往目标方向走。”
```

当前：

```yaml
PathFollowCritic:
  cost_power: 1
  cost_weight: 5.0
  offset_from_furthest: 25
  threshold_to_consider: 2.8
```

### 太弱

现象：

- 障碍附近容易停住
- 宁愿待在低 cost 区域
- 不愿继续向前

### 太强

现象：

- 为了前进过于激进
- 障碍附近容易“硬冲”

---

# 13. PathAlignCritic

作用：

> 让局部轨迹靠近全局路径。

当前：

```yaml
PathAlignCritic:
  cost_power: 2
  cost_weight: 8.0
  max_path_occupancy_ratio: 0.05
  trajectory_point_step: 2
  threshold_to_consider: 0.40
  offset_from_furthest: 25
```

### `cost_weight` 太大

表现：

- 死贴全局路径
- 动态障碍前不愿绕
- 绕障时犹豫 / 减速
- 很难主动偏离 global path

### `cost_weight` 太小

表现：

- 轨迹很自由
- 绕障容易
- 但回到全局路径较慢
- 空旷区域可能走得比较散

---

## `cost_power`

当前：

```yaml
cost_power: 2
```

这意味着大偏差的惩罚会比 `cost_power: 1` 增长更快。

直观上：

```text
小偏差：还能接受
大偏差：惩罚明显变强
```

所以它可能让机器人：

- 平时允许轻微偏离
- 偏得太多时强力拉回来

但“转弯变好”不是固定结论，仍然取决于 PathFollow、Obstacle、速度和前视距离。

---

# 14. PathAlign 与动态避障

这是比赛里非常重要的一点：

```text
ObstacleCritic
     ↑
     |
PathFollow —— PathAlign
```

三者必须平衡。

如果 PathAlign 太强：

```text
全局路径上出现动态障碍
        ↓
MPPI 想绕开
        ↓
但偏离 global path 的 cost 很高
        ↓
犹豫 / 减速 / 停住
```

如果需要动态障碍绕行能力：

```text
不要把 PathAlign 权重压得过高
```

---

# 15. `max_path_occupancy_ratio`

作用：

> 当全局路径附近有较多障碍占据时，降低 PathAlign 对“死贴路径”的要求。

这是动态避障非常重要的机制。

当前：

```yaml
max_path_occupancy_ratio: 0.05
```

如果这部分触发正确，机器人遇到路径上的障碍时应该更愿意偏离 global path。

---

# 16. ObstaclesCritic

当前：

```yaml
ObstaclesCritic:
  cost_power: 1
  repulsion_weight: 0.1
  critical_weight: 20.0
  consider_footprint: false
  collision_cost: 50.0
  collision_margin_distance: 0.11
  near_goal_distance: 0.5
```

理解：

```text
critical_weight
= 快撞了时的强惩罚

repulsion_weight
= 平时离障碍多远
```

---

## `repulsion_weight`

当前：

```yaml
repulsion_weight: 0.1
```

属于非常低的避障排斥。

优点：

- 不容易因为 inflation 就大幅减速
- 敢走窄路
- 高速感明显

风险：

- 更靠近障碍
- 动态障碍出现时余量可能较小

---

## `collision_margin_distance`

当前：

```yaml
collision_margin_distance: 0.11
```

表示在碰撞边界外额外增加约 11 cm 的危险区域。

调大：

```text
更保守
```

调小：

```text
更敢贴障碍
```

它必须小于机器人实际必须通过的最窄通道剩余裕量。

---

## `collision_cost`

当前：

```yaml
collision_cost: 50.0
```

这个值建议重点检查当前 Jazzy 源码的默认值和实际使用逻辑。

如果你使用的是官方 Jazzy ObstaclesCritic，碰撞相关 cost 通常应该是一个很强的惩罚。

如果设得过小，可能出现：

```text
碰撞轨迹与普通高 cost 轨迹区分度不足
```

所以后续建议直接对照你当前工作空间中的：

```text
nav2_mppi_controller
```

源码确认。

---

# 17. Inflation 与 ObstaclesCritic

Obstacle 行为不能只看 Critic。

必须一起看：

```text
inflation_radius
cost_scaling_factor
repulsion_weight
robot footprint
```

常见规律：

```text
inflation 范围越大
→ repulsion_weight 往往需要越小
```

否则：

```text
影响范围大
+
惩罚又强
=
机器人特别怕 costmap
```

---

# 18. Omni + TwirlingCritic

当前：

```yaml
wz_max: 0.1
wz_std: 0.1

TwirlingCritic:
  twirling_cost_weight: 10.0
```

已经非常强烈地限制 yaw 行为。

如果你的导航设计就是：

> 底盘朝向基本无所谓，优先 x/y 平移

那么这种思路可以。

如果以后换成舵轮或希望底盘姿态跟随路径，就不能继续照搬这套配置。

---

# 19. Visualize 调试

调试阶段：

```yaml
visualize: true
publish_optimal_trajectory: true
```

建议在 RViz 中重点看：

- MPPI 候选轨迹
- optimal trajectory
- transformed global plan
- local costmap
- footprint
- dynamic obstacles

观察：

```text
“MPPI 有没有采到正确轨迹？”
```

和：

```text
“采到了但没选？”
```

是完全不同的问题。

---

## 如果候选轨迹根本没有绕过去

检查：

```text
std 太小
batch_size 太小
prediction horizon 太短
motion model 不合适
速度 / 加速度约束太死
```

---

## 如果候选轨迹明明有好轨迹，但最后不选

检查：

```text
Critic 权重
Obstacle / PathFollow / PathAlign 平衡
costmap cost
GoalCritic
```

---

# 20. 高频调试命令

查看 controller 参数：

```bash
ros2 param dump /controller_server
```

单独查看控制频率：

```bash
ros2 param get /controller_server controller_frequency
```

查看 cmd_vel：

```bash
ros2 topic echo /cmd_vel
```

查看发布频率：

```bash
ros2 topic hz /cmd_vel
```

查 MPPI 是否加载的是工作空间版本：

```bash
ros2 pkg prefix nav2_mppi_controller
```

如果输出：

```text
/opt/ros/jazzy
```

说明正在使用 apt 版本。

如果输出：

```text
.../sim_origion_nav/install/nav2_mppi_controller
```

说明当前使用工作空间编译版本。

---

# 21. 推荐调试顺序

不要同时改一堆 Critic。

建议严格按以下顺序：

```text
1. controller_frequency / model_dt
                ↓
2. time_steps / prediction horizon
                ↓
3. vx_max / vy_max / wz_max
                ↓
4. ax / ay / az
                ↓
5. vx_std / vy_std / wz_std
                ↓
6. local costmap 尺寸 + inflation
                ↓
7. ObstaclesCritic
                ↓
8. PathFollowCritic
                ↓
9. PathAlignCritic
                ↓
10. GoalCritic
                ↓
11. Twirling / Deadband 等特殊 Critic
```

---

# 22. 常见现象 → 优先排查

| 现象 | 优先看 |
|---|---|
| 跑不到最大速度 | `vx_std`、加速度、prediction horizon、costmap |
| 起步很慢 | `vx_std`、`ax_max`、PathFollow |
| 起步太猛 | `ax_max`、`vx_std` |
| 左右蛇形 | Obstacle / inflation / PathAlign |
| 一碰 inflation 就减速 | `repulsion_weight` 太大 |
| 宁可停也不走 | Obstacle 太强 / PathFollow 太弱 |
| 太贴障碍 | repulsion 太弱 / collision margin 太小 |
| 窄路不敢走 | collision margin / inflation 太大 |
| 动态障碍前不绕 | PathAlign 太强 |
| 绕障后不回路径 | PathAlign 太弱 |
| 明明有好轨迹却不选 | Critic 权重 |
| 根本采不到好轨迹 | std / batch / horizon |
| Omni 莫名自转 | `wz_std`、`wz_max`、TwirlingCritic |
| 转弯差 | PathAlign、PathFollow、std、horizon、wz |
| 快到终点很早就慢 | GoalCritic threshold |
| CPU 撑不住 | batch_size、time_steps、visualize、footprint |

---

# 23. 当前这套参数需要重点复查的地方

当前：

```yaml
time_steps: 56
model_dt: 0.06
vx_max: 3.0
vy_max: 3.0
ax_max: 20.0
ay_max: 20.0
prune_distance: 8.5
```

重新计算：

```text
prediction horizon = 3.36 s
最大纯 x 预测距离 ≈ 10.08 m
```

所以原配置中的这些旧注释应删除或更新：

```text
0.05 * 56 = 2.8 s
2.8 * 3 = 8.4 m
prune_distance >= 8.4 m
```

它们已经不对应当前参数。

---

# 24. 建议记录实验结果

每次只改一类参数，并记录：

```text
日期：
场景：
参数改动：

controller_frequency:
model_dt:
time_steps:
batch_size:

vx_max:
vy_max:
wz_max:

vx_std:
vy_std:
wz_std:

ax_max:
ay_max:

Obstacle:
PathFollow:
PathAlign:

现象：
最高速度：
转弯：
避障：
窄路：
动态障碍：
是否蛇形：
是否贴墙：
CPU：
```

推荐用 Git commit 管理：

```bash
git add nav2_params.yaml
git commit -m "mppi: increase vx/vy std to 0.5"
```

不要一次改十几个参数，否则测试后无法知道到底是谁起作用。

---

# 25. 一句话理解整个 MPPI 调参

```text
先保证“看得对”
→ prediction horizon / costmap

再保证“跑得出来”
→ speed / acceleration / std

再保证“不会撞”
→ inflation / ObstaclesCritic

再保证“愿意继续走”
→ PathFollowCritic

再保证“别走太散”
→ PathAlignCritic

最后再处理：
Goal / Twirling / Deadband / 特殊行为
```

对于 RoboMaster 高速动态导航，最需要长期平衡的是：

```text
        Obstacles
            ↕
PathFollow ←→ PathAlign
```

- Obstacles 太强：怕障碍、减速、停住
- PathFollow 太强：为了前进过于激进
- PathAlign 太强：死贴全局路径，不愿动态绕障
- PathAlign 太弱：轨迹很自由，但绕完不愿回来

---

# 26. 参考资料

Nav2 Jazzy MPPI Controller 官方文档：

https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/mppi_controller/configuring_mppic/

Nav2 源码：

https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_mppi_controller

建议所有最终参数以你当前实际编译的 Jazzy `nav2_mppi_controller` 源码为准，避免直接照搬 Humble / Rolling / 第三方博客的字段与默认值。
