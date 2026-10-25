# Adaptive-LIO 论文对照改造报告（2026-10-10）

## 范围与基线

保持纯 LiDAR+IMU、Ceres、内外双层 ESKF。内层是唯一权威后验，外层安装包含状态、重力、18×18 P、传播配置及 Q 的值快照，再重放 IMU。未改导航、FakeBaselink、TF 拓扑或外部话题；未引入轮速、第三滤波器、24维状态或1ms激光批次。

附件 ZIP SHA256 与说明相符。本地相对附件的六个文件差异已保留：两份 YAML 的 9.805 加速度换算与关闭副雷达话题、main 可选副雷达订阅、timer 两文件全调用统计、lidarodom 阶段计时。详见 `validation/paper_alignment/attachment-local.diff`。R0 提交冻结用户当前包，不把旧上游 e0339963 当成本轮实现起点。包外未提交改动保持原状。

发现并纠正说明中的静态推断：YAML 虽指定 CT，但旧 init() 会在 CONSTANT_VELOCITY 下强制 POINT_TO_PLANE。因此不能断言默认运行路径已经重复补偿，也不能宣称其是实车误差主因。本轮删除了隐式覆盖和可冲突路径，以显式时间契约取代。

## 实现与删除

| 轮次 | 改动 | 核心位置 |
|---|---|---|
| R0 | 代码包哈希、差异、当前核心测试与独立分支 | validation/paper_alignment/R0.md |
| R1 | 原始点保留，新增帧末 IMU 工作点；覆盖不足显式剔除；唯一 IMU_DESKEW_END 路径；实际 Ceres 因子差分 | point_time.h、cloudMap.hpp、lidarodom.cpp、lidarFactor.h |
| R2 | 候选只写当前帧；后验提交位置/姿态/速度/零偏，刷新世界点；拒绝不更新地图、关键帧与接受时间 | frame_commit.h、ProcessMeasurements、poseEstimation |
| R3 | 最终实际对应的6维信息、尺度归一化、谱/秩/弱子空间；完整矩阵R共同进入门控与Joseph更新 | lidar_geometry_quality.h、lidar_observation.h、match_quality.h、eskf.hpp |
| R4 | 有界方向协方差、零空间大方差、候选到先验创新坐标的切空间搬运；fixed默认 | geometry配置、geometry_covariance.md |
| R5 | 换算后统一IMU质量判定，保存轴掩码与处理策略；有限保持可选、默认关闭；保守区间合并、可重复重放、双层故障通知 | imu_quality.h、imu.h、main_ada.cpp、imu_odom_predictor.h、imu_odom_output.h |
| R6 | 质量筛选后按空间/法向确定性分桶，候选最多3倍残差预算，保留原500上限；阶段计时与覆盖统计 | lidar_sampling.h、addSurfCostFactor、Timer |

删除旧 CT/iterative/no-compensation 路径、手写未调用激光因子和旧 parameterization、备用 addSurfCost、多余 mmap 初始化、旧3维法向反馈接口/队列/参数。删除已不再读取的 all_state_frame 历史缓存和帧末重复拷贝；current_state 只保留后验派生状态。移除无用标量/6向量 ObserveSE3 重载。CMake 使用安装的 Ceres 配置，移除不存在的 /opt/ceres-2.2 硬编码。

`map_update_rotation_trigger` 按 YAML 中的 rad 解释；旧代码误把返回度数的 AngularDistance 与其比较。本轮改为 SO(3) 对数范数。优化收敛参数 `thres_orientation_norm` 仍保持原有度数单位。

## 状态与点流

```
原始主/副 LiDAR 点 + 采样秒时间
    → 副雷达校正时间与裁剪、T_main_aux
    → 冻结 IMU 预测轨迹，去畸变到帧末 IMU 工作点
    → Ceres 帧末临时候选 → 最终对应几何诊断/R → 创新门控
    → 内层 ESKF 原子更新 → 提交后验派生状态 → 发送完整值快照
    → 按后验刚体刷新世界点 → 地图/ScanContext/注册点云
完整快照 + 同一带质量标记的 IMU 历史 → 外层重放 → odom/TF
```

seed 无配准观测，不调用 ObserveSE3、不伪造几何信息，仅用有效内层状态注册地图与建立新鲜度锚点。rejected 保留传播先验、不入图、不发候选注册点云、不前移最近接受时间。下一帧从最新内层先验重新匹配。aux fusion=false 时副雷达只追加到发布点云，不进入匹配、几何诊断、地图或ScanContext。

去畸变使用冻结预测轨迹，后验刚体注册不代表历史轨迹平滑重估。同一激光观测不会被反复融合。比较内外层时必须先传播到相同时间，日志不再拿主机墙钟减 bag 传感器时间当延迟。

## 配置迁移

两份 mapping_m YAML 已迁移，保留用户的 gnorm、话题、外参及延迟配置。

- 新增 `point_time_model: IMU_DESKEW_END`，旧 `icpmodel/motion_compensation/point_to_plane_with_distortion/beta_*` 报迁移错误。
- 删除无作用的 `map_options`、`preprocess.degenerate_window_num`；旧项报错。
- `lidar_covariance_mode: fixed` 默认使用原数值方差 `0.01 I6`。`adaptive` 实现可用，但参数未经真实标定，不能作为已验收默认模式。
- `geometry.reference_length_m` 固定归一化长度；floor/ceiling 为归一化坐标方差，residual_variance_m2 为残差尺度，correlation_inflation 应在独立数据上标定。
- `match_max_translation` 单位m，`match_max_rotation` 单位rad，均为同一时刻先验—候选硬上界，不因增大R而放开。保留支持数、代价与NIS门控。
- IMU量程以三轴SI值提供，0表示未知/关闭该轴饱和检测，不猜测硬件量程。`hold_seconds: 0` 默认异常即停止，非零≤50ms是未经实车验收的实验保持；恢复样本数也受该预算限制。
- `diagnostic_stride` 控制接受帧日志；seed/拒绝另记录。阶段耗时采用 steady_clock，不等于端到端延迟。

## 实际验证

详细记录见 `validation/paper_alignment/`，全部为本次执行；历史附件日志未当作本次证据。

- `bash test/run_core_tests.sh`：原有回归、完整R、几何退化、IMU质量及重放。
- `bash test/run_registration_tests.sh`：链接实际 Ceres AutoDiff 因子，验证Eigen四元数manifold与物理右扰动。
- `colcon build --packages-select adaptive_lio --cmake-args -DBUILD_TESTING=ON`：ROS Jazzy全量编译。
- `ctest --test-dir build/adaptive_lio -R '^adaptive_lio_(core|geometry|registration|pipeline|imu_quality)$' --output-on-failure -V`：五组确定性测试，包含真实ProcessMeasurements种子/接受/拒绝/恢复事务。
- `test/replay_smoke.py`：独立ROS域、localhost、仅启动被测LIO和bag，不启动导航或硬件驱动；验证odometry/TF、时间、有限值、四元数、注册发布和故障状态；保存节点日志、轨迹与阶段耗时。

无噪声合成世界点复原最大误差9.93014e-16 m；实际因子局部雅可比差2.52344e-10；非小角创新搬运差4.12995e-10。场景有效秩3/5/6。真实处理事务的后验注册点浮点误差约1.2e-7 m。测试不是RViz肉眼判断，也不是准确率或实车安全性验收。

R5首轮编译遇到glog限频宏在单行if中的作用域错误，已加大括号修复；保留r5-build.log和成功retry日志。依赖与旧源码警告未冒充零警告构建。

## 未验收事项

自适应R与短时IMU保持均未完成真实传感器/独立回放标定，因此默认不开启。现有bag无提供真值、场景标签、实际量程、CPU/控制器延迟预算，不能宣称绝对精度提升、统计置信度已校准，或覆盖急加减速、高速自旋、走廊及遮挡恢复全部工况。

用户随后要求停止 bag 回放，后续仅做编译与确定性测试；R6最终采样版未继续跑包，其耗时与实车验收由用户完成。多平面采样仅报告确定性合成覆盖收益，不能写成速度提升。地图数据项仍与IMU先验和历史地图相关；6维R是工程近似。现有PCL发布下采样在这份bag上有体素索引范围警告（基线也存在），详见回放记录；这不影响本轮对后验提交与拒绝帧入图的断言。

## 最终验收结果

- `final-core.log`：全部核心、几何与IMU质量断言通过。
- `final-registration.log`：实际Ceres因子和去畸变测试通过。
- `r6-build.log`：最终ROS2构建成功（有旧源码/依赖警告）。
- `final-ctest.log`：5/5通过；包含支持不足拒绝与优化成功后硬创新门控拒绝，再次接受恢复。
- R6确定性构造的偏置候选集，在同样60个残差预算下，顺序截取秩3，空间/法向分桶秩6；相反法向不重复分桶。这个构造用例不能替代最终版本的实车性能对照。
- 用户要求停止bag后，仅执行编译和确定性测试。已结束的R0/R5各45秒回放保存在 `validation/paper_alignment/replay_completed_before_stop/`；最终R6版本未回放，不能套用R5耗时。
- `git diff --check` 通过。用户原有包外导航配置、启动/回放脚本及运行日志保持原状。

可复制的非bag测试命令见 `test/README.md`。分轮提交见 `validation/paper_alignment/commits.txt`。
