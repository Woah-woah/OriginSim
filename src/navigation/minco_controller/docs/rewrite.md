# 任务：继续完善 OriginSim 的 Recovery MPC 初始碰撞脱困功能

你现在位于 `OriginSim` 项目的 `nav` 分支。

请直接阅读当前工作区源码，在**现有未提交修改的基础上继续开发**，完成 Recovery 初始碰撞状态下的 MPC 闭环脱困。

本任务不是重新设计 Recovery，而是完成当前已经进行到一半的实现。

**重要要求：**

1. 当前工作区中的代码是唯一修改基准，不要用 GitHub 远程版本覆盖本地修改。
2. 禁止执行 `git reset`、`git checkout`、`git restore` 等可能覆盖工作区修改的命令。
3. 必须先阅读完整相关代码，确认已有修改，再进行剩余实现。
4. 不要重新实现 RecoveryPlanner，也不要更换已经确定的脱困算法。
5. 不要引入新的规划框架、行为树、状态机或第三方依赖。
6. 尽量复用已有类、函数和成员变量。
7. 保持代码简单，避免不必要的抽象和复杂 C++ 写法。
8. 直接完成代码修改、编译检查和必要测试，不要只输出修改建议。
9. 不得为了让机器人移动而直接关闭所有碰撞检测。
10. 如果某个安全约束无法可靠实现，必须保留停车拒绝行为，并明确说明原因，不得伪装成实现成功。

---

# 一、项目现状

当前项目使用：

- ROS 2 Jazzy
- 自定义 MapServer / MapClient
- EDT 距离场
- Footprint 几何碰撞检测
- MINCO + MPC 局部控制器
- PathExecutor 统一发布 `/cmd_vel`

当前 Recovery 的目标流程：

```text
导航出现卡住或局部路径不可执行
          ↓
TaskManager 触发 Recovery
          ↓
PathExecutor 停车并确认停稳
          ↓
RecoveryPlanner 搜索脱困目标
          ↓
生成 Recovery Path
          ↓
将 Recovery Path 交给 MPC
          ↓
MPC 低速闭环跟踪
          ↓
机器人逐渐脱离初始障碍
          ↓
完全脱困并停稳
          ↓
恢复正常碰撞约束
          ↓
TaskManager 对原目标重新规划
          ↓
继续正常导航
```

此前 Recovery 采用手动计算的匀速 `cmd_vel`。

目前已经开始将它改为：

**RecoveryPlanner 负责目标搜索，MincoMpcController 负责闭环运动。**

---

# 二、已经确定的 RecoveryPlanner 算法

不要修改总体思路。

RecoveryPlanner 使用：

1. EDT 梯度生成候选脱困方向。
2. Footprint 顶点附近 EDT 梯度补充方向。
3. 原路径反方向作为候选方向。
4. 16 个均匀采样方向作为补充。
5. 沿各方向从近到远搜索。
6. 每轮优先考虑最近距离内的可行目标。
7. 对起点已经碰撞的情况，允许尝试脱困路径。
8. 对正常起点继续使用严格的 `checkMotion()`。
9. 终点要求完全脱离障碍并满足 `min_goal_clearance`。
10. 保留原有候选评分、路径长度和搜索超时机制。

其中：

```cpp
countCollisionCells()
```

负责计算 Footprint 接触的障碍栅格数量。

```cpp
checkEscapePath()
```

负责检查从起点到终点的碰撞栅格数量是否单调不增，并且终点完全脱离障碍。

这两个函数目前可能仍然位于 `recovery.cpp` 中。

`TrajectorySafety` 中也已经加入或正在加入相似的公共接口：

```cpp
collisionCellCount()
checkEscapeMotion()
```

请检查这些函数的实际实现。

如果功能重复且结果一致，统一复用 `TrajectorySafety`，避免长期维护两套不同的 Footprint 碰撞计数算法。

**不要改变基于碰撞栅格数量变化的脱困路径筛选逻辑。**

---

# 三、MPC 已经完成的部分

重点检查：

```text
src/navigation/minco_controller/include/minco_controller/minco_mpc_controller.hpp

src/navigation/minco_controller/src/minco_mpc_controller.cpp
```

此前已经进行了以下修改：

### 1. setPlan() 接受 Recovery 参数

预期接口：

```cpp
void setPlan(
    const nav_msgs::msg::Path & path,
    double recovery_speed = 0.0,
    double recovery_acceleration = 0.0);
```

正常导航保持：

```cpp
controller_.setPlan(path);
```

Recovery 使用：

```cpp
controller_.setPlan(mpc_path, c.speed, c.acceleration);
```

Recovery 的参考速度和加速度应使用传入参数。

同时将 Recovery 速度和加速度限制直接设置到 MPC 求解器中。

不得只在 MPC 求解后截断速度，否则会破坏 MPC 优化结果和加速度约束。

### 2. Recovery 模式

已经增加：

```cpp
bool recovery_mode_{false};
```

以及 Recovery 相关状态变量。

检查：

- `reset()` 正确清除 Recovery 状态。
- 正常导航的 `setPlan()` 恢复普通 MPC 配置。
- Recovery 结束后不会继续使用低速参数。
- `MpcSolver::setConfig()` 不会造成不符合预期的状态重置。
- MPC 发布零速度时，上一条控制指令状态能够正确同步。

### 3. MPC 内部死区

Recovery 模式使用：

```cpp
command_model.deadzone =
    recovery_mode_ ? 0.0 : deadzone_;
```

避免小加速度下控制器被软件死区卡住。

普通导航的死区行为保持不变。

---

# 四、已经实现的初始碰撞处理

重点检查：

```text
src/navigation/minco_controller/include/minco_controller/trajectory_safety.hpp
```

此前已经添加或修改：

```cpp
collisionCellCount()
checkEscapeMotion()
sampleMotion()
sampleEnvelope()
samplePatches()
checkMotion()
checkPatches()
checkEnvelope()
```

其中 Recovery 使用：

```cpp
std::unordered_set<size_t> recovery_contacts_;
```

保存初始已经接触的障碍栅格。

原定逻辑：

```text
Recovery 开始
      ↓
记录初始碰撞栅格集合
      ↓
冻结集合
      ↓
MPC 优化
      ↓
允许已有接触逐步消失
      ↓
禁止产生新的接触栅格
      ↓
碰撞数量不得增加
      ↓
完全脱离后恢复严格碰撞检测
```

请特别检查以下问题：

### 4.1 初始接触集合不能逐周期扩大

不能每个控制周期重新把当前接触的障碍加入白名单。

正确行为：

- 在本次 Recovery 运动开始之前记录初始接触集合。
- 整次 Recovery 使用固定的初始集合。
- 每个控制周期重新检测实际接触情况。
- 发现新接触栅格立即拒绝该运动并停车。
- 不允许在已经完全脱离后重新进入初始接触栅格。
- 若地图几何发生变化，使栅格索引不再对应原有空间位置，必须拒绝继续使用旧白名单。
- 如果初始化接触集合失败，禁止启动运动。

现有代码如果在 MPC 第一次求解时才冻结集合，需要检查该时刻是否可能已经发生运动。

优先在 `PathExecutor` 停稳并确定 Recovery 起点时完成初始化，再将该集合交给 MPC。

### 4.2 碰撞栅格数量必须单调不增

沿预测路径依次检测：

```text
5 → 5 → 4 → 3 → 1 → 0  允许

5 → 4 → 5              拒绝

5 → 4 → 3 → 3 → 0      允许
```

这里的数字表示 Footprint 接触的障碍栅格数量。

采样间距：

```cpp
std::min(0.025, map_resolution * 0.5)
```

保留已有采样预算上限。

需要同时处理路径上的平移与实际旋转。

### 4.3 禁止接触新的障碍

仅比较数量并不足以防止接触新障碍。

例如：

```text
起点接触障碍 A：1 个栅格

下一位置接触障碍 B：1 个栅格
```

数量未增加，但产生了新接触。

因此使用之前已经设计的初始接触集合，确保连续运动不接触集合之外的障碍栅格。

注意：

- 接触数量检查用于判断脱困趋势。
- 固定接触集合用于限制新碰撞。
- 两者都必须保留。
- 不能用简单的障碍数量判断替代完整几何检查。

### 4.4 对已有接触的处理

`samplePatches()` 中已经设计了：

```cpp
initial_contacts
```

对于初始接触集合中的障碍，允许暂时不生成分离平面，使优化器能够尝试离开重叠区域。

但必须检查这是否会导致优化器允许机器人进一步深入障碍物。

仅跳过分离平面并不能保证安全。

需要通过最终轨迹验证阻止：

- 新接触；
- 碰撞数量增加；
- 脱离后重新进入碰撞；
- 地图越界；
- 制动过程中的接触恶化。

不能确认这些条件时拒绝控制指令。

### 4.5 正常导航不能受影响

当 `recovery_mode_ == false` 时：

```cpp
checkPose()
checkMotion()
checkPrediction()
sampleMotion()
sampleEnvelope()
checkEnvelope()
```

必须维持原来的严格碰撞约束。

Recovery 特殊处理只能在明确激活的 Recovery 模式中生效。

---

# 五、完成 MPC 最终输出安全检查

重点修改：

```cpp
MincoMpcController::computeVelocityCommands()
```

目前需要确认以下阶段都正确适配 Recovery：

### 阶段 A：当前位姿

获取当前 Footprint 接触栅格数量。

正常导航：

```cpp
field.checkPose(observed, ...);
```

Recovery 初始重叠：

允许进入特殊求解流程，但必须验证当前接触集合合法。

如果已经完全脱离，必须恢复严格检查。

### 阶段 B：控制延迟

检查：

```cpp
observed → delayed
```

其中：

```cpp
delayed.x = observed.x + observed.vx * control_delay_;
delayed.y = observed.y + observed.vy * control_delay_;
```

还需要考虑角速度对应的姿态变化。

不能只验证 MPC 预测结果，忽略控制延迟期间实际车辆的运动。

### 阶段 C：MPC 预测轨迹

保留：

```cpp
checkPrediction()
```

Recovery 模式要求：

- 每段碰撞数量单调不增；
- 不接触新障碍；
- 不越界；
- 已经无碰撞的后续路段恢复严格检查。

### 阶段 D：最终控制指令

保留：

```cpp
field.checkEnvelope()
```

Recovery 模式使用对应的初始接触处理，但仍然要验证指令保持期间的运动。

不能因为允许初始重叠，就跳过整段指令包络检查。

### 阶段 E：制动过程

基于当前实测速度、控制延迟、指令保持周期和保守制动能力，检查紧急停车可能经过的区域。

必须考虑机器人在接收到停车指令前仍会继续运动。

不得仅使用理想情况下的瞬时停车模型。

无法保证制动过程不会产生新的危险接触时，拒绝发布非零速度。

### 阶段 F：输出

只有上述检查全部通过，才允许：

```cpp
return command;
```

普通导航依旧走原来的严格检查分支。

MPC 控制器本身不得直接发布 `/cmd_vel`。

---

# 六、完成 PathExecutor 的 MOVING 阶段接入

这是当前最重要的剩余工作。

打开：

```text
src/nav_executor/src/path_executor.cpp
```

重点修改：

```cpp
PathExecutor::Impl::tickRecovery()
```

此前已经修改了 STOPPING 阶段，允许起点碰撞时继续调用：

```cpp
RecoveryPlanner::plan()
```

并且计划将搜索结果通过：

```cpp
auto mpc_path = recoveryPath(
    recovery_plan_.selected,
    pose.header);

controller_.setPlan(
    mpc_path,
    c.speed,
    c.acceleration);
```

交给 MPC。

首先检查以上代码是否已经存在，以及是否正确。

不要重复添加。

## 6.1 保存 Recovery 初始碰撞状态

此前 `initial_collision` 是 STOPPING 阶段的局部变量。

但 MOVING 阶段每个周期仍然需要知道当前 Recovery 是否从初始重叠状态启动。

因此需要在 `PathExecutor::Impl` 中保存本次 Recovery 的状态。

例如：

```cpp
bool recovery_initial_overlap_{false};
```

具体命名可以保持当前代码风格，但不能只依靠 STOPPING 内部的局部变量。

状态必须在：

- `startRecovery()` 初始化；
- Recovery 规划成功时确定；
- `completeRecovery()` 清除；
- `stop()` 清除；
- `fail()` 的 Recovery 结束路径中正确清除。

不得让下一次正常导航继承上一次 Recovery 的特殊权限。

## 6.2 修改 MOVING 前面的严格碰撞检查

当前代码中可能仍然存在：

```cpp
safety.checkPose(
    observed,
    "Recovery 实时轮廓");
```

```cpp
safety.checkMotion(
    observed,
    inertia,
    "Recovery 延迟及惯性制动扫掠");
```

```cpp
safety.checkMotion(
    observed,
    target,
    "Recovery 剩余轨迹实时复检");
```

这些调用会导致初始重叠时直接失败。

修改要求：

**正常 Recovery：**

保持上述严格检查。

**初始重叠 Recovery：**

使用之前完成的专用脱困验证：

```cpp
collisionCellCount()
checkEscapeMotion()
```

结合固定初始接触集合，对实时位姿、剩余轨迹和制动范围进行检查。

但不能直接删除原有检查，也不能简单地通过 `if (initial_collision)` 绕过所有安全约束。

若地图更新导致新的障碍接触、接触数量恶化或制动无法验证，必须停车并拒绝继续运动。

## 6.3 保留原有运动保护

以下机制必须保留：

- Recovery 总超时；
- 最大累计行驶距离；
- 单次最大行驶距离；
- 停车确认；
- 实测速度超限检查；
- 定位跳变检查；
- 里程计连续性检查；
- 地图健康状态检查；
- 地图生产端重启检测；
- 控制计算超时；
- 轨迹横向偏差检查；
- 无进展超时；
- 急停；
- 发布前 TF 和地图有效性检查。

不要因为更换 MPC 而删掉这些逻辑。

## 6.4 使用 MPC 输出

MOVING 阶段应通过：

```cpp
auto output =
    controller_.computeVelocityCommands(
        pose, speed).twist;
```

获取控制指令。

注意：

`computeVelocityCommands()` 已经输出机器人坐标系速度。

禁止再次错误地执行坐标变换。

最终仍然通过：

```cpp
publishCommand(output);
```

统一发布。

不要在 MPC 外再次人为截断控制速度。

如果速度超限，应当拒绝该指令，并检查 MPC 内部的速度约束配置。

## 6.5 终点判断

Recovery 不是接近目标就立即报告成功。

必须满足：

```text
进入目标位置容差
       ↓
发布停车指令
       ↓
实际速度下降
       ↓
确认停稳
       ↓
确认 Footprint 完全脱离障碍
       ↓
严格碰撞检查通过
       ↓
Recovery 成功
```

继续复用已有：

```cpp
RecoveryPhase::FINISHING
```

不要新增其他状态机。

在 FINISHING 中保持：

```cpp
safety.checkPose(observed, ...);
```

只有真正安全停稳，才能调用：

```cpp
completeRecovery(true, ...);
```

---

# 七、正常导航恢复

Recovery 成功后：

1. 停止发送 Recovery 速度。
2. 清除 MPC Recovery 模式。
3. 清除初始接触集合。
4. 清除 Recovery 特殊状态。
5. 保留 TaskManager 原来的导航目标。
6. 触发原目标重新规划。
7. 正常导航重新使用原始 MPC 参数。
8. 正常导航恢复完整严格碰撞检测。

不要修改 TaskManager 原有的目标管理和重规划设计，除非确实发现接入所需的明确错误。

---

# 八、必须修复的潜在问题

阅读当前代码时，重点检查以下问题：

### 1. Recovery 模式没有正确重置

确保 `reset()`、`setPlan()`、`completeRecovery()` 之间状态一致。

### 2. MPC 速度约束没有真正生效

检查 `solver_->setConfig()` 后，求解器使用的是否确实是 Recovery 速度和加速度限制。

正常导航必须能够恢复原有参数。

### 3. MPC 优化器内部仍然拒绝初始重叠

检查：

```cpp
sampleMotion()
sampleEnvelope()
samplePatches()
```

不能只修改最终 `checkPrediction()`。

### 4. 初始接触集合被重新扩大

禁止把新碰到的障碍加入白名单。

### 5. 完全脱离后又重新发生碰撞

完全脱离后永久恢复严格检查，直到本次 Recovery 结束。

### 6. MPC 与 PathExecutor 重复处理同一个控制过程

允许保留必要的独立安全复检，但避免两套互相矛盾的控制算法。

尤其不要保留旧的匀速运动代码与 MPC 同时计算控制指令。

### 7. 旧的制动逻辑干扰 MPC

旧 Recovery 中的手动：

```cpp
desired
delta
recovery_command_
recovery_braking_
```

可能与 MPC 控制冲突。

检查并清理不再需要的匀速控制部分。

但必须保留等效的停车、超时、累计距离和紧急制动安全保障。

### 8. 规划成功但没有发布可视化

保留：

```cpp
visualizeRecovery(pose.header);
```

必须能够发布：

```text
/nav_executor/recovery_candidates
/nav_executor/recovery_path
```

不要修改现有话题名称和消息类型。

---

# 九、明确禁止修改的内容

除非为修复本任务直接引起的编译错误，否则不要修改：

- EDT 距离场算法；
- MapServer 动态障碍差分算法；
- 全局规划器；
- TaskManager 的任务调度策略；
- 普通导航 MPC 的轨迹跟踪算法；
- 普通导航碰撞检查策略；
- ROS Topic 名称；
- TF 坐标系定义；
- 串口协议；
- 其他非 Recovery 模块。

不要增加新的 YAML 参数。

Recovery 使用现有：

```yaml
recovery:
  speed: 0.18
```

以及已有 MPC 配置。

不要增加第二套独立的速度、加速度和碰撞余量参数。

不要自动修改 `recovery.enabled` 的默认值。只报告当前启动配置中是否启用 Recovery。

---

# 十、实现和验收要求

必须完成以下工作：

### 1. 阅读代码

先完整阅读：

```text
src/nav_executor/src/recovery.cpp
src/nav_executor/include/nav_executor/recovery.hpp

src/nav_executor/src/path_executor.cpp

src/navigation/minco_controller/src/minco_mpc_controller.cpp
src/navigation/minco_controller/include/minco_controller/minco_mpc_controller.hpp

src/navigation/minco_controller/include/minco_controller/trajectory_safety.hpp

src/navigation/minco_controller/src/mpc_solver.cpp
src/navigation/minco_controller/include/minco_controller/mpc_solver.hpp
```

确认当前代码状态，再修改。

### 2. 修改代码

直接在现有代码上完成。

不要只提供伪代码。

### 3. 编译

执行：

```bash
colcon build \
  --packages-up-to nav_executor \
  --symlink-install
```

如有编译错误，定位并修复。

### 4. 测试

至少验证以下场景：

| 场景                 | 预期结果              |
| ------------------ | ----------------- |
| 起点没有碰撞             | Recovery MPC 正常求解 |
| 起点碰撞，存在可脱离方向       | 能生成候选目标和受约束的脱困运动  |
| 碰撞数量增加             | 拒绝对应运动            |
| 接触新的障碍栅格           | 拒绝对应运动            |
| 完全脱离后再次碰撞          | 严格拒绝              |
| 已有初始接触但无法安全制动      | 拒绝非零指令            |
| 地图更新出现新的障碍         | 重新检查，无法确认安全则停车    |
| Recovery 目标到达但仍有碰撞 | 不能报告成功            |
| Recovery 安全停稳      | 成功并触发原目标重规划       |
| Recovery 失败        | 停车，不继续发送运动指令      |
| 正常导航               | 原 MPC 行为不受影响      |

优先编写或复用现有单元测试。若没有现成仿真环境，不得声称已经验证实车运动。

### 5. 最终告知

完成后只需告知：

1. 修改了哪些文件；
2. 各文件修改的具体内容；
3. Recovery MPC 的实际运行流程；
4. 初始碰撞如何处理；
5. 哪些安全约束仍然保留；
6. 编译是否通过；
7. 测试结果；
8. 尚未解决的问题。

特别说明是否已经验证“初始重叠时真正产生非零 MPC 指令”，不要把“规划成功”或“进入 MOVING 状态”当成“实际脱困成功”。

---

# 最终目标

我需要的是：

**一个能够在仿真中处理初始 Footprint 重叠、搜索脱困目标、利用 MPC 进行低速闭环跟踪，并在脱困结束后恢复正常导航的 Recovery 系统。**

必须延续当前已经实现的设计：

```text
EDT 方向搜索
    ↓
从近到远选择脱困目标
    ↓
Footprint 碰撞数量检查
    ↓
固定初始接触集合
    ↓
MPC 低速跟踪
    ↓
预测轨迹和新接触检查
    ↓
控制延迟与制动安全验证
    ↓
终点严格碰撞检查
    ↓
正常导航恢复
```

不要重新设计架构，不要引入新的规划器。

**请先检查当前工作区代码，然后直接完成剩余实现。**
