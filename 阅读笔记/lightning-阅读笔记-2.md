# Lightning-LM 读码清单（可勾选）

> **用法**：把「完成」列的 `[ ]` 改成 `[x]` 即打勾；每站末尾有「过关标准」，答不上来就别往下走。
> **行号基线**：当前工作区 `HEAD = 1e11924`（等于上游 `568dc4b` 之后仅注释级改动），行号已核对；带 `≈` 的是函数体起点估算，跳不准就用函数名搜索。
> **建议节奏**：站 0–2 半天，站 3 一天半，站 4 半天，站 5 一天，站 6–7 各半天。

## 进度总览

| 站 | 主题 | 预计 | 完成 |
|---|---|---|---|
| 0 | 准备与基线 | 0.5 h | `[ ]` |
| 1 | 骨架：消息怎么进系统 | 1 h | `[ ]` |
| 2 | 数据结构（全局词典） | 1.5 h | `[ ]` |
| 3A | LIO 入口与两条线的汇合 | 1.5 h | `[ ]` |
| 3B | IMU 初始化 / 预积分 / 去畸变 | 1.5 h | `[ ]` |
| 3C | ESKF 迭代更新与观测模型 | 2 h | `[ ]` |
| 3D | 关键帧与局部地图（iVox） | 1.5 h | `[ ]` |
| 4A | 回环检测与位姿图 | 1 h | `[ ]` |
| 4B | g2p5（3D→2D 栅格） | 1 h | `[ ]` |
| 4C | 地图落盘与加载 | 1 h | `[ ]` |
| 5A | 定位门面 + NDT 匹配 | 2 h | `[ ]` |
| 5B | PGO 滑窗 + 高频输出 | 1.5 h | `[ ]` |
| 6 | miao 最小集 | 1.5 h | `[ ]` |
| 7 | `568dc4b` diff 专题 | 2 h | `[ ]` |

---

## 站 0：准备与基线

| 完成 | 做什么 | 命令 / 位置 | 过关自检 |
|---|---|---|---|
| `[ ]` | 备一份 bag（NCLT 或 Livox db3），确认能跑 | `ros2 run lightning run_slam_offline --input_bag X --config ./config/default_nclt.yaml` | 能跑出 `/data/new_map/` |
| `[ ]` | 修好 IDE 跳转（含 `.vscode/c_cpp_properties.json`） | 见前文 IntelliSense 方案 | `Eigen/Core` 无红线 |
| `[ ]` | 收集日志热点 | `grep -rn 'Timer::Evaluate' src/` | 知道哪几个函数是计时热点 |
| `[ ]` | 记住 4 个"枢纽文件" | `laser_mapping.cc`、`slam.cc`、`localization.cpp`、`nav_state.h` | 能说出各自职责 |

---

## 站 1：骨架 —— 消息怎么进系统

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `src/app/run_slam_offline.cc` | 16–19, 35–51, 53–75 | flag 定义、`SlamSystem` 构造、三个 handler 注册、`SaveMap` | 离线与在线在"消息来源"上差在哪？ |
| `[ ]` | `src/core/system/slam.h` | 38–50, 81–103 | `Options`、成员模块与订阅句柄 | 系统持有哪些模块？谁管生命周期？ |
| `[ ]` | `src/core/system/slam.cc` | 24–124 | `Init`：各模块创建条件（lio/loop/ui/g2p5/ros node） | 关掉 `with_loop_closing` 会影响哪些模块？ |
| `[ ]` | `src/core/system/slam.cc` | 137–232 | `SaveMap`：`GetGlobalMap`→`TiledMap`→`global.pcd` / `map.pgm` / `map.yaml` | `map.yaml` 的 `resolution` 从哪来？（坑：写死 0.05） |
| `[ ]` | `src/core/system/slam.cc` | 234–303 | `ProcessIMU` / `ProcessLidar`（KF 级联到回环、g2p5、UI） | 为什么只有"新关键帧"才通知下游？ |
| `[ ]` | `src/wrapper/bag_io.cc` + `.h` | cc:14–37；h:63–107 | 顺序读包、按话题精确分发、`flg_exit` | 为什么"话题名写错"会静默无反应？ |
| `[ ]` | `src/core/system/loc_system.cc` | 19–72, 74–98 | 定位节点：订阅、TF 广播、`loc_started_` 门控 | TF 是哪个模块、哪个频率发的？ |

**过关标准**：不看代码画出「bag/ROS → SlamSystem → (LIO / LoopClosing / g2p5 / UI)」框图，并指出唯一的地图落盘点。

---

## 站 2：数据结构（全局词典）

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `src/common/point_def.h` | 17, 120 | `PointXYZIT`（`time` 单位 ms）、`G_m_s2` | 点云里的 `time` 是相对谁的？（答：帧内相对） |
| `[ ]` | `src/common/imu.h` / `measure_group.h` | imu:≈15；mg:17 | `IMU` 三元组、`MeasureGroup` 的 begin/end/imu | 一帧雷达对应哪些 IMU 字段？ |
| `[ ]` | `src/common/nav_state.h` | 22–28, 54–95, 102–126, 145–167 | 12 维与 4 个索引、`get_f`/`df_dx`/`df_dw`、`oplus`(**L93 速度积分被注释**)、`boxplus`、`Getba()=0` | 为什么删 `ba/grav/offset`？谁替代？为什么速度不积分？ |
| `[ ]` | `src/common/nav_state.cc` | MetaInfo 表定义 | `vect_states_` / `SO3_states_` | 这两张表在 ESKF 里被谁遍历？ |
| `[ ]` | `src/common/keyframe.h` | 17–74 | KF 的字段与互斥锁、`pose_lio_` vs `pose_opt_` | `SetLIOPose` 为什么同时改 `pose_opt_`？ |
| `[ ]` | `src/common/options.h` / `options.cc` | h:19–101；cc:12–69 | 全局参数与默认值（PGO 噪声写死在这） | 为什么 `pgo:` 的 yaml 段无效？ |
| `[ ]` | `src/io/yaml_io.h` | 28–48 | `GetValue<node,key>` 与断言 | 缺 key 时会发生什么？（答：抛异常，被 catch 吞掉） |
| `[ ]` | `src/common/params.h/.cc` | 全文略读 | 确认它是**死代码**（23 维时代的遗留） | 有谁 include 它？（答：没有） |

**过关标准**：能说明 `NavState::dim=12`、`full_dim=12`、`process_noise_dim=12` 三个数字的关系，以及 `oplus` 与 `boxplus` 的区别。

---

## 站 3A：LIO 入口与两条线的汇合（**最关键**）

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `laser_mapping.h` | 33–50, 118–143, 146–227 | `Options`、私有函数清单、缓冲队列与 `kf_/kf_imu_/state_point_` | `kf_` 与 `kf_imu_` 各代表什么时刻？ |
| `[ ]` | `laser_mapping.cc` | 38–133 | `LoadParamsFromYAML`（含 `lidar_type` 1–4、iVox 邻域、外参、协方差） | 哪些键缺了会直接让 `Init` 失败？ |
| `[ ]` | `laser_mapping.cc` | 141–167 | `ProcessIMU`：回绕清缓冲、`kf_imu_.Predict`、入队 | `kf_imu_` 的更新为什么只服务 UI/高频？ |
| `[ ]` | `laser_mapping.cc` | 412–477 | 三个 `ProcessPointCloud2` 重载 | Livox 与 PointCloud2 的时间戳来源分别是什么？ |
| `[ ]` | **`laser_mapping.cc`** | **479–544** | **`SyncPackages`**：取最旧帧、`lidar_end_time_` 算法、IMU 覆盖判据 | 为什么取最旧而不是最新？`lidar_end_time_` 怎么算？ |
| `[ ]` | `pointcloud_preprocess.cc` | 35–92（Livox）、94+、≈140+、≈180–247 | 四种雷达的 per-point 时间、ROI/盲区/抽稀 | `time_scale` 对哪种雷达生效？ |

**过关标准**：能说出「一帧点云从进 buffer 到成为 `measures_`」的完整条件和时间戳计算式，并指出 `point_num` 相关的越界/空帧风险。

---

## 站 3B：IMU 初始化 / 预积分 / 去畸变

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `imu_processing.hpp` | 82–109 | 构造与 `Reset`（`Q_` 对角） | `Q_` 最后三个 0 对应哪个噪声？ |
| `[ ]` | `imu_processing.hpp` | 124–172 | `IMUInit`：均值/方差、`grav_`、`bg_`、初始 P | 初始化需要多少 IMU？重力方向怎么定？ |
| `[ ]` | `imu_processing.hpp` | 315–357 | `Process`：初始化门槛、**加速度单位判别（336–344）** | ‖a‖≈1 与 ‖a‖≈9.8 分别代表什么？ |
| `[ ]` | `imu_processing.hpp` | 174–313 | `UndistortPcl`：前向预积分调 `Predict`、外推到帧尾、**后向逐点补偿** | 为什么是"从帧尾往回补偿"？ |
| `[ ]` | `imu_filter.h` | 55+ | 可选的陀螺去尖峰 | 什么时候必须关掉它？（斜装 30° 那个 issue） |
| `[ ]` | `pose6d.h` | 12–30 | `Pose6D` 缓存了什么 | 去畸变时怎么在 IMU 之间插值？ |

**过关标准**：能画出 IMU 线的数据流：`imu_buffer_ → measures_.imu_ → Predict 循环 → imu_pose_ → 逐点补偿`。

---

## 站 3C：ESKF 迭代更新与观测模型

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `eskf.hpp` | 25–27, 33–40, 51–64, 80+ | 维度常量、`ObsType`、`CustomObservationModel`、各 thresholds | `HTH_` 为什么是 6×6 而不是 12×12？ |
| `[ ]` | `eskf.cc` | 15–32 | `SymmetrizeAndFloorCovariance`（P 对称化/上下限/NaN） | 这些"trick"解决什么数值问题？ |
| `[ ]` | `eskf.cc` | 38–95 | `Predict`：`f_x`/`f_w` 装配、SO3 块的 `A_matrix`、P 传播与膨胀 | 速度不积分后，加速度对均值还有影响吗？ |
| `[ ]` | `eskf.cc` | 105–200 | `Update` 主循环、`dx` 的 SO3 映射、`P_` 行列旋转 | 为什么每次迭代都重新线性化？ |
| `[ ]` | `eskf.cc` | 199–248 | `HTH` 特征分解 → 可观测掩码 → 信息形式更新 | 退化时被"掩掉"的方向会发生什么？ |
| `[ ]` | `eskf.cc` | 250–277 | NaN 直接 return（**隐患**）、步长拒绝 | 两条错误路径分别怎么恢复状态？ |
| `[ ]` | `eskf.cc` | 279–356 | AA 分支、收敛判定、P 更新、(45) 式 | 收敛条件是什么？ |
| `[ ]` | `laser_mapping.cc` | 613–760 | `ObsModel`：iVox kNN → `esti_plane` → 点面残差与 H | 有效点数门槛是多少？ |
| `[ ]` | `laser_mapping.cc` | 763–798 | 点-点 ICP 残差与权重 | 点点/点面维度不同，怎么拼进 6×6？ |
| `[ ]` | `laser_mapping.cc` | 169–330 | `Run` 串流程（首帧、跳帧、降采样、`Update`、KF 判定） | 点数不足时的降级策略是什么？ |

**过关标准**：能只凭记忆写出「点位残差 → HTH/HTr → dx → boxplus」的公式链，并说出退化处理发生在哪一步。

---

## 站 3D：关键帧与局部地图

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `laser_mapping.cc` | 364–410 | `MakeKF`：判据、`pose_opt_` 链式递推、`MapIncremental` 调用点 | 定位模式下 KF 的额外判据是什么（2 s）？ |
| `[ ]` | `laser_mapping.cc` | 545–612 | `MapIncremental`：体素中心自适应插入 | 为什么"只在 KF 更新"？帧间配准用哪张图？ |
| `[ ]` | `ivox3d.h` | 53, 101–104, 213–236, 258–282, 285–287 | Options、LRU 结构、邻域枚举、`AddPoints`、`Pos2Grid` | 内存上限由什么保证？ |
| `[ ]` | `ivox3d.h` | 133–196 | `GetClosestPoint`（kNN + `max_num`） | 为什么是"近似 kNN"？ |
| `[ ]` | `ivox3d_node.hpp` | 94–99, 117–181 | 每体素 10 点 FIFO、`KNNPointByCondition` | 每体素保留多少点、怎么淘汰？ |
| `[ ]` | `laser_mapping.cc` | ≈870 | `GetProjCloud`（关键帧投影点云，给定位用） | 里面最多拼几个 KF、每帧多少点？ |

**过关标准**：能解释「iVox 与 ikd-Tree 的区别」以及 `ivox_grid_resolution` / `kf_dis_th` 如何共同影响配准质量。

---

## 站 4A：回环检测与位姿图

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `loop_closing.cc` | 27–70 | `Init`：miao 配置、信息矩阵、yaml、在线线程 | 增量模式怎么开的？ |
| `[ ]` | `loop_closing.cc` | 72–143 | `HandleKF` → `DetectLoopCandidates`（纯几何候选） | 没有描述子，靠什么找候选？（坑） |
| `[ ]` | `loop_closing.cc` | 168–251 | `ComputeForCandidate`：子图构建、**10/5/2/1 多分辨率 NDT**、`Tij` | 初值用的是 LIO 位姿还是优化位姿？ |
| `[ ]` | `loop_closing.cc` | 253–348 | `PoseOptimization`：顶点/运动边/高度先验/回环边/`Optimize(20)`/outlier | 有没有固定顶点？`SetLevel(1)` 有效吗？ |
| `[ ]` | `miao/core/types/vertex_se3.h`、`edge_se3.h` | 20–31 / 27–43 | `OplusImpl`、误差定义与雅可比来源 | 误差是数值雅可比还是解析？ |

**过关标准**：能说清「回环如何改变 `pose_opt_` 并触发 g2p5 重绘」，以及为什么 `with_height` 对多楼层有害。

---

## 站 4B：g2p5（3D→2D）

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `g2p5.h` | 45–61, 63–124 | Options 与线程/地图成员 | 前端渲染与后端重绘各由谁触发？ |
| `[ ]` | `g2p5.cc` | 273–346 | `Convert3DTo2DScan`：地面平面、360 射线、障碍打黑 | 高度带 `[min_th_floor, max_th_floor]` 的作用？ |
| `[ ]` | `g2p5.cc` | 366–432 | 逐射线选距离 + `SetWhitePoints` 打白 | "能从上方穿过、不能从下方穿过"体现在哪？ |
| `[ ]` | `g2p5.cc` | 434–482 | `DetectPlaneCoeffs`（RANSAC 地面） | 估计失败时的回退是什么？ |
| `[ ]` | `g2p5_grid_data.h`、`g2p5_subgrid.h` | 12–19 / 16, 26–54 | `GridData`、子网格与高度感知更新 | occupancy 阈值在哪算？ |
| `[ ]` | `g2p5_map.cc` | 245–306, 308–371 | `ToROS` / `ToCV` | 两个坑：`flip(...,1)` 方向、`resolution` 写死 |

**过关标准**：能解释为什么"栅格地图不用于定位、只用于显示/输出"。

---

## 站 4C：地图落盘与加载

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `tiled_map.h` | 37–46, 48–66, 127–242, 254–262 | 动态策略、Options、API、`Pos2Grid` | 一个 chunk 多大？动态图层的三种策略差别？ |
| `[ ]` | `tiled_map.cc` | 13–38, 40–81 | `ConvertFromFullPCD` / `SaveToBin` | `index.txt` 格式？（含 FP 段；origin 恒 0） |
| `[ ]` | `tiled_map.cc` | 83–170 | `LoadMapIndex`（**L144 全量加载**）、`_dyn.pcd` 加载 | README 说的"分区动态加载"真的生效吗？ |
| `[ ]` | `tiled_map.cc` | 276–362 | `LoadOnPose`（加载/卸载，**L329 静态卸载被注释**） | 大场景内存会怎样增长？ |
| `[ ]` | `tiled_map.cc` | 440–540, 596–608 | `UpdateDynamicCloud`、`FallsInDynamicArea` | 关闭多边形时"什么算动态"？（坑：全部） |

**过关标准**：能列出 `data/new_map/` 下每个文件的来历与用途。

---

## 站 5A：定位门面 + NDT 匹配

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `localization.h` / `.cpp` | h:78–120；cc:19–138 | 三条流水线、两条异步队列、输出回调 | 为什么离线/在线要走不同的分发路径？ |
| `[ ]` | `localization.cpp` | 140–232 | `ProcessLidarMsg` → `LidarOdomProcCloud`（LO + DR + `GetProjCloud`） | `loc_on_kf_` 为 true 时定位多久跑一次？ |
| `[ ]` | `localization.cpp` | 234–297 | `LidarLocProcCloud`、`ProcessIMUMsg`（DR 100 Hz） | `GetIMUState()` 失败时会怎样？ |
| `[ ]` | `lidar_loc.cc` | 24–48, 59–129 | NDT 参数、yaml 键、动态策略、功能点初始化、地图线程 | 哪些配置项读了但没用？ |
| `[ ]` | `lidar_loc.cc` | ≈440–780 | `Align`：初值 → `LoadOnPose` → `Localize` → **0.1 修正（630–632）** → `force_2d` | 为什么只取 10%？（设计意图） |
| `[ ]` | `lidar_loc.cc` | 367–410, 822–882 | `UpdateGlobalMap`（**重建 NDT 丢参数**）、`Localize`（**每帧写 tgt.pcd**、`loc_success` 恒真） | 三个坑分别怎么修？ |
| `[ ]` | `localization_result.cc` | 10–36 | `ToGeoMsg`（`map`→`base_link`）、`ToNavState` | TF 的父/子坐标系是谁定的？ |

**过关标准**：能回答"定位失败的三种表现"以及为什么 `FOLLOWING_DR/FAIL` 状态不可达。

---

## 站 5B：PGO 滑窗 + 高频输出

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `pgo_impl.h` | 28, 95–118 | `PGOFrame`、`Options`（噪声默认值） | `PGO_MAX_FRAMES` 是多少？ |
| `[ ]` | `pgo_impl.cc` | 230–302 | `RunOptimization`、`BuildProblem`、`AddVertex`（ID 回收） | 为什么顶点 ID 会被改写？ |
| `[ ]` | `pgo_impl.cc` | 304–400 | `AddLidarLocFactors`（绝对）/ `AddLidarOdomFactors`（相对） | 各自用了什么鲁棒核？ |
| `[ ]` | `pgo_impl.cc` | ≈402–495, ≈788–855 | `AddDRFactors`（**未调用**）、prior、滑窗、`Marginalize`（**占位**） | 窗口丢弃的信息去哪了？ |
| `[ ]` | `pgo.cc` | 28–141 | `PubResult`：置信度检查 → 外推 → 平滑 → 回调 | 低置信度连续出现会怎样？ |
| `[ ]` | `pgo.cc` | 354–469 | `ExtrapolateLocResult`：DR 外推 + 时间对齐 | `latest_time` 怎么取？ |
| `[ ]` | `smoother.h` | 27–102 | `PushDRPose` / `PushPose`（增益 0.01、>2 m→0.2、>5 m 跳变） | 平滑与"高精度"如何折衷？ |
| `[ ]` | `pose_extrapolator.h/.cc` | h:25–84；cc:11–134 | 确认**整个类是死代码** | 真正的高频路径在哪？（答：`pgo.cc:354`） |

**过关标准**：能画出「NDT 结果（KF 级）→ PGO（5 帧）→ DR 外推 → 平滑 → TF（IMU 频率）」的时序。

---

## 站 6：miao 最小集

| 完成 | 文件 | 行号 | 读什么 | 自检问题 |
|---|---|---|---|---|
| `[ ]` | `miao/core/types/vertex_se3.h`、`edge_se3.h`、`edge_se3_prior.h`、`edge_se3_height_prior.h` | ≈20–31 / 27–43 / 22–35 / 17–30 | 顶点更新与三类边的误差 | 高度先验的测量值是几？（答：0） |
| `[ ]` | `miao/core/graph/optimizer.h` | 43, 59, 148–149, 153–169 | 两个 `InitFrom*`、增量队列 | 增量模式禁止什么？ |
| `[ ]` | `miao/core/graph/optimizer.cc` | 21–113 | `InitializeOptimization` / `InitFromRaw` / `InitFromLast` | "不重建模型"具体省了什么？ |
| `[ ]` | `miao/core/graph/optimizer.cc` | 155–200, 286–374 | `Optimize` 循环、**环形顶点替换**、`RemoveEdge/RemoveVertex` | 顶点回收时边怎么办？ |
| `[ ]` | `miao/core/solver/block_solver.hpp` | 70–83, 265–355, 483–517 | `BuildStructure`（增量）、`BuildSystem`（全量重线性化） | 每轮真的全量线性化吗？ |
| `[ ]` | `miao/core/opti_algo/opti_algo_lm.cc` | 24–138 | LM 主循环、λ 初始化与增益比 | 失败步如何回滚？ |
| `[ ]` | `miao/core/graph/graph.cc`、`sparse/sparse_block_matrix.hpp` | 98–118 / 88–96 | `RemoveEdge`（**erase 后解引用**）、`Block() const`（**拼写 bug**） | 两处 bug 各在什么场景触发？ |

**过关标准**：能用一句话说清 miao 的"增量"到底增量在哪、以及它与 iSAM2 的本质差别。

---

## 站 7：`568dc4b` diff 专题

| 完成 | 命令 | 目标 | 自检问题 |
|---|---|---|---|
| `[ ]` | `git show --stat 568dc4b` | 建立改动全貌（23 文件 +378/−446） | 哪三个文件占了一半改动量？ |
| `[ ]` | `git show -w 568dc4b -- README.md` | **先读变更日志**（含"只用陀螺计算 lio"的原因） | 为什么速度积分被注释？ |
| `[ ]` | `git show -w 568dc4b -- src/common/nav_state.h src/common/nav_state.cc src/core/lio/eskf.hpp` | 23 维 → 12 维、S2 删除 | 删掉的四块各自替代方案？ |
| `[ ]` | `git show -w 568dc4b -- src/core/lio/eskf.cc` | 33 处 S2 摘除、P 保护 tricks | `f_w_final` 现在多少维？ |
| `[ ]` | `git show -w 568dc4b -- src/core/lio/imu_processing.hpp src/core/lio/laser_mapping.h src/core/lio/laser_mapping.cc` | `offset_*_lidar_` → `_fixed_`、点点 ICP、`GetProjCloud` | 外参变成固定后，谁负责它？ |
| `[ ]` | `git show -w 568dc4b -- src/core/localization/lidar_loc/lidar_loc.cc` | `guess_from_self`/`try_self_extrap` 被改写 | 为什么这两个配置现在无效？ |
| `[ ]` | `git show -w 568dc4b -- src/core/localization/localization.cpp src/core/localization/pose_graph/` | `GetProjCloud` 接入、pgo 小补丁 | 定位为何改为"关键帧 map-to-map"？ |
| `[ ]` | `git show -w 568dc4b -- src/ui config` | UI 去 grav/ba 显示、三套配置调参 | 三份配置各调了什么？ |
| `[ ]` | `git show 568dc4b:src/core/lio/eskf.cc \| grep -n vel_clip` | 验证 README 与代码的矛盾 | 该信 README 还是信代码？ |

**过关标准**：能不看代码讲出"这次稳定性更新改了什么、为什么改、遗留了什么"。

---

## 附录 A：陷阱清单（读到就打勾，边读边验）

| 完成 | 陷阱 | 位置 |
|---|---|---|
| `[ ]` | `oplus` 不积分速度 | `nav_state.h:93` |
| `[ ]` | `Update(obs, R)` 的 R 是信息标量（传 1.0），不是协方差 | `eskf.cc:105` / `laser_mapping.cc:267` |
| `[ ]` | 局部地图只在关键帧更新 | `laser_mapping.cc:389,545` |
| `[ ]` | 同步取 buffer 最旧帧 | `laser_mapping.cc:479` |
| `[ ]` | KF 存的是去畸变**全量**点云（注释写反） | `laser_mapping.cc:365` / `keyframe.h:68` |
| `[ ]` | LIO 从不设置 `confidence_` / `lidar_odom_reliable_` | `lidar_loc.cc:210` |
| `[ ]` | 定位结果只取 10% | `lidar_loc.cc:630-632` |
| `[ ]` | `UpdateGlobalMap` 重建 NDT 丢参数 | `lidar_loc.cc:368-394` |
| `[ ]` | 每帧写 `./data/tgt.pcd` | `lidar_loc.cc:848-851` |
| `[ ]` | `Localize` 恒返回成功 | `lidar_loc.cc:853-857` |
| `[ ]` | `pgo:` 配置整段无效 | `options.cc:51-63` vs `config/*.yaml` |
| `[ ]` | `try_self_extrap` / `YawSearch` 无效 | `lidar_loc.cc:296,585-594` |
| `[ ]` | `Marginalize` 是占位、`AddDRFactors` 未调用 | `pgo_impl.cc:846,402` |
| `[ ]` | `PoseExtrapolator` 整个死代码 | `pose_extrapolator.cc` |
| `[ ]` | miao `RemoveEdge` erase 后解引用 | `graph.cc:106-113` |
| `[ ]` | `SparseBlockMatrix::Block() const` 拼写 bug | `sparse_block_matrix.hpp:95` |
| `[ ]` | AA 的 `% D` 应为 `% m`（越界） | `anderson_acceleration.h:75` |
| `[ ]` | 静态地图全量常驻、卸载被注释 | `tiled_map.cc:144,329` |
| `[ ]` | `map.yaml` 分辨率写死 0.05 | `slam.cc:214` |
| `[ ]` | `SaveMap` 无空检查且 `remove_all` 目标目录 | `slam.cc:157,169,178` |
| `[ ]` | 离线 livox 话题硬编码 `/livox/lidar` | `run_slam_offline.cc:68` / `run_loc_offline.cc:59` |
| `[ ]` | `livox_ros_driver2` 未注册为 ament 包（`ros2 interface list` 报错） | `thirdparty/livox_ros_driver/CMakeLists.txt:134-137` |

## 附录 B：五个验收问题（全部答上来 = 读通）

| 完成 | 问题 |
|---|---|
| `[ ]` | 一帧点云从进队到姿态输出，IMU 在哪几步被用到？ |
| `[ ]` | 关键帧何时产生？地图何时长大？回环如何纠正已有关键帧？ |
| `[ ]` | 定位的初值来自哪里、结果如何被"打折"、又如何在 100 Hz 输出？ |
| `[ ]` | 哪些 yaml 配置是真生效的，哪些是装饰？（列出至少 5 个无效键） |
| `[ ]` | miao 的增量优化省了什么、没省什么？ |

## 附录 C：随身命令

```bash
# 定位函数 / 调用点
grep -rn "函数名" src/ --include=*.cc --include=*.h
# 函数级历史（解释"为什么被注释"）
git log -L :函数名:文件路径 --oneline
# 计时热点
grep -rn 'Timer::Evaluate' src/
# 单文件 diff（读 568dc4b 时必加 -w）
git show -w 568dc4b -- <file>
# 跟着日志读（有 bag 时）
ros2 run lightning run_slam_offline --input_bag X --config ./config/default_nclt.yaml 2>&1 | grep -E "\[ mapping \]|LIO state|create kf"
```

---

需要的话，我可以把这份清单再压成「**只有 30 个必读点**的最小版」（适合 1 天速通），或者反过来扩成带**每个函数该记录什么笔记**的模板表（适合做长期维护文档）。
