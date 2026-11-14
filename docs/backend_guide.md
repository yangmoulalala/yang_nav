# ZLIO 定位后端使用与机制说明

本文说明 `src/z-localization/backend`（节点 `backend_node`）当前的工作方式：怎么启动、有哪些状态、什么时候触发重定位、主要参数的含义，以及在仿真测试中已知的问题。

- 话题总表见 [topics_and_usage.md](topics_and_usage.md)。
- 全部参数（含离线工具）见 `src/z-localization/backend/docs/parameters.md`。
- 仿真回归测试见 `tools/sim_tests/README.md`。

恢复流程、参考帧回退与 TF 接线已于 2026-11-14 按当前源码复核。当前行为：UWB 不再融合进因子图，只在 `SUSPECT`/`LOST` 局中恢复时作位置先验；开机初始化只在起点或启动提示附近 2 m 内搜索；不做全局重定位；删除了恢复运动门（第 4.2、4.5 节）。

---

## 1. 后端做什么

前端 ZLIO 只给出局部里程计（`lio_odom →` 前端 `frames.imu`，仿真 `mid360_link`、真车 `mid360_imu_link`），会慢慢漂移。后端负责把它放进全局地图坐标系 `map`：

| 模式 | 做什么 | 输出 |
| --- | --- | --- |
| 建图 `mapping` | 用关键帧建位姿图，Scan Context 检索回环，iSAM2 优化；可保存地图包 | `map → lio_odom`，`map` 原点 = 前端起点 |
| 定位 `localization` | 加载不可变的先验地图包，周期性做 scan-to-map 配准修正漂移；丢失时重定位 | `map → lio_odom`，`map` = 先验地图坐标系 |

主输入是三个话题：`/frontend/frame`（去畸变点云 + 位姿）、`/frontend/odometry`（高频预测）、`/frontend/session`（前端身份、代次、中断/失败通知）。`uwb.enabled` 时另订阅裁判系统 UWB：`/uwb_pose`（位置）和 `/uwb_pose/team`（己方颜色），只用于局中恢复（第 4.2 节）。

主要输出：

| 输出 | 说明 |
| --- | --- |
| TF `map → lio_odom → base_link` | 组合启动由后端在同一 TFMessage 中发布两条动态边，导航首选 |
| `/backend_node/localization_pose` | map 系车体位姿 + 协方差 |
| `/localization/odometry` | map 系车体位姿 + 车体系速度，高频 |
| `/backend_node/localization_state` | 状态文本，含 `pose_trusted`（transient local） |
| `/localization/lio_world_cloud` | map 系实时点云 |

---

## 2. 启动

### 2.1 一键启动（推荐）

```bash
cd ~/yang_nav && source install/setup.bash

# 仿真（先启动 Gazebo；测定位时建议不开 gui，RTF 才能到 1.0）
ros2 launch sentry_gazebo sim.launch.py
ros2 launch bringup sim.launch.py lidar:=dual mode:=mapping rviz:=true

# 真车
ros2 launch bringup real.launch.py lidar:=single mode:=mapping
```

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `mode` | `mapping` | `mapping` 建图；`localization` 加载先验地图 |
| `map_dir` | `~/yang_nav/maps/sim`（真车 `maps/real`） | `save_map` 写入的目录；`localization` 且未给 `prior_map` 时从这里加载 |
| `prior_map` | 空 | 指定其他先验地图目录（绝对路径） |
| `rviz` | `false` | 启动后端 RViz；可视化话题开关只由参数文件的 `visualization.*` 决定 |
| `lidar` | 仿真 `dual`，真车 `single` | 是否融合 L2 |

配置文件：仿真 `bringup/config/sim/backend.yaml`，真车 `bringup/config/real/backend.yaml`。二者都是完整参数文件（不再继承，键集合须与 `backend/config/example.yaml` 一致，用 `python3 tools/check_bringup_params.py` 检查），由 `localization.launch.py` 作为 `params_file` 传入。车体外参不在其中，运行期间按消息时间戳从 TF（URDF + `joint_state_publisher`）查询 `common.sensor_frame ← common.body_frame`，容差由 `common.body_extrinsic_tolerance` 给出。

### 2.2 单独启动后端

前端已在运行时，可以只启动/重启 backend 包的 launch（调参时只重启它）。用 bringup 的参数文件即可与全套启动一致。该配置开启 `common.publish_odom_tf`，前端必须关闭 `output.publish_tf`；若前端独立发布 TF，则改用后端 `common.publish_odom_tf: false` 的完整参数文件：

```bash
ros2 launch backend backend.launch.py \
  params_file:=$(ros2 pkg prefix bringup)/share/bringup/config/sim/backend.yaml use_sim_time:=true \
  mode:=localization prior_map:=/abs/map save_dir:=/abs/save rviz:=true
```

`backend.launch.py` 参数：`params_file`（默认包内 `config/example.yaml`）、`use_sim_time`、`mode`（只有 `mapping` / `localization`）、`prior_map`（`localization` 必填）、`save_dir`（写入 `trajectory.save_directory`，留空不可保存）、`rviz`。`prior_map.enabled`、`prior_map.directory`、`trajectory.save_directory` 只由 launch 提供，不写入 yaml。单独运行时 TF 中须已有 `common.sensor_frame ← common.body_frame`。

### 2.3 建图 → 保存 → 定位

```bash
# 1. 建图：绕场地开，回到走过的地方触发回环；在各处原地转一圈（见 7.1）
ros2 launch bringup sim.launch.py mode:=mapping rviz:=true
# 2. 保存地图包（keyframes.txt、clouds/、sc_descriptors.bin、map.pcd）
ros2 service call /backend_node/save_map std_srvs/srv/Trigger
# 3. 定位
ros2 launch bringup sim.launch.py mode:=localization rviz:=true
```

- 同一 `map_dir` 再次保存会覆盖；要保留旧图请换目录。
- 定位模式下 `save_map` 会返回失败（先验地图不可变，不累积运行地图）。
- `maps/` 已加入 `.gitignore`。

### 2.4 启动前检查

上一次 launch 若被强制关闭，Python 节点可能残留，导致 `/lidar/points` 有两个发布者，前端收到重复帧、定位明显变差。启动前先确认：

```bash
pgrep -af 'merge_node|zlio_node|backend_node'           # 应为空
ros2 topic info /lidar/points                            # 启动后 Publisher count 应为 1
```

---

## 3. 定位状态

| 状态 | 含义 | `pose_trusted` | 输出 |
| --- | --- | --- | --- |
| `INITIALIZING` | 还没有全局位姿 | false | 不发布位姿、世界点云 |
| `NORMAL` | 正常 | true | 正常发布 |
| `OBS_DEGRADED` | 全局观测与预测冲突，但未达阈值 | true | 继续用里程计外推输出 |
| `SUSPECT` | 怀疑出错，正在局部重定位 | false | 沿用最后可信 `map→lio_odom`，由里程计前推（`tf_frozen=false`）；`/localization/odometry` 保留前端速度（已转到车体系），位姿协方差放大到至少 10 m / π。前端重置跨代次后旧 `map←odom` 不再适用，退回冻结（同 `LOST`） |
| `LOST` | 丢失，等待重定位（需要新鲜 UWB，UWB 位置附近 2 m） | false | 冻结在最后可信位姿（`tf_frozen=true`）；`/localization/odometry` 车体速度清零、协方差放大 |

**导航必须订阅 `localization_state`，`pose_trusted=false` 时停车或降级。** `SUSPECT` 前推的精度取决于进入 `SUSPECT` 前的全局位姿是否正确：误判无效（如 7.2）时输出仍准，绑架或错误恢复后会沿着偏掉的 `map←odom` 继续走。`LOST` 冻结期间车如果还在动，输出位姿与真实位姿的误差会随时间增大（实测原地 0.6 rad/s 自转时每 0.2 s 增加约 8°，该数据测于 SUSPECT 也冻结的旧实现）。

`localization_state` 是分号分隔的 `key=value`：

```
status=NORMAL;pose_trusted=true;tf_frozen=false;tf_published=true;epoch=0;generation=5;pose_revision=..;stamp_ns=..;estimator_failure_count=0;last_estimator_failure=;reason=...
```

### 3.1 状态转移规则

代码：`backend/src/localization_state.cpp`。

```
                     重定位成功（需连续确认）
  INITIALIZING ──────────────────────────────▶ NORMAL ◀──────┐
                                               │  ▲          │ 观测被接受
                       单次观测冲突            │  │          │
                 NORMAL ──────────▶ OBS_DEGRADED ─┘          │
                                               │             │
   ① 单次观测模型无效                          │ 连续冲突 ≥ 7 │
   ② 速度异常           ─────────────────────▶ SUSPECT ──────┤ 局部重定位成功
   ③ OBS_DEGRADED 冲突达阈值                   │             │
                                               │ 连续失败 ≥ 3│
                                               ▼             │
   估计器失败 / 前端输入中断 ───────────────▶ LOST ──────────┘ 重定位成功
```

| 事件 | 条件 | 结果 |
| --- | --- | --- |
| 全局观测被接受 | 配准有效且 NIS 检验通过（p ≥ `innovation.rejection_probability`=0.001） | 清零计数，回到 `NORMAL` |
| 观测冲突 | 配准有效但与预测不一致（NIS 检验失败，或校正超过 `tracking.max_translation_correction` 1.5 m / `max_rotation_correction_deg` 15°） | 计数 +1；未到 `innovation.conflicts_before_suspect`（7）为 `OBS_DEGRADED`，到了进 `SUSPECT` |
| 严重冲突 | 冲突且配准对应比例 ≥ `innovation.severe_min_support_ratio`（0.6），并触发校正上限或 NIS p < `severe_rejection_probability`（1e-8）。配准可靠而修正极端，典型于绑架或错误恢复 | 连续冲突中累计 `severe_conflicts_before_suspect`（2）次即进 `SUSPECT`，不必等 7 次（日志 `(severe 2/2)`） |
| 恢复试用期内严重冲突 | 恢复提交后 `recovery.probation_observations`（5）次全局观测内 | 一次即进 `SUSPECT`（日志 `during recovery probation`） |
| **观测模型无效** | 配准不收敛、信息矩阵无效、或 rmse 超出局部模型范围 | **一次就进 `SUSPECT`**（不计数） |
| 速度异常 | 只在有先验地图时检查：线速度 > `tracking.suspicious_linear_speed`（10 m/s）或角速度 > `suspicious_angular_speed`（30 rad/s） | 进 `SUSPECT` |
| `SUSPECT` 下重定位失败 | 连续 `recovery.suspect_failures_before_lost`（3）次（源点云帧数不够、没有新鲜 UWB、结果有歧义都不计；有历史帧但都被累积门限排除算一次） | 进 `LOST` |
| 估计器失败 / 前端输入中断 | 例如前端报告 `reset_required` 或 `failed` | 进 `LOST` |
| 重定位成功 | 见第 4 节 | 回 `NORMAL` |

“观测无效”（不能构造似然）与“观测冲突”（能构造，但和预测不符）是两条不同的路径：前者没有缓冲，这是第 7.2 节问题的来源。

---

## 4. 重定位触发机制

代码：`backend/src/localization_recovery.cpp`、`recovery_matcher.cpp`、`relocalization_*.cpp`。

### 4.1 何时运行

- 状态是 `INITIALIZING`、`SUSPECT` 或 `LOST`，且加载了先验地图时，维护周期运行恢复流程；`NORMAL`/`OBS_DEGRADED` 下不运行重定位，只做常规跟踪。没有先验地图时这条恢复流程不运行：纯里程计在收到首帧时直接进 `NORMAL`（`localization.cpp`）。
- 调度间隔 `recovery.period`（0.2 s），且必须有新的同步帧。
- 源点云：累积最近 `recovery.source.min_frames`～`max_frames`（2～5）帧、时间跨度 ≤ 0.5 s、平移 ≤ 1.5 m。帧数不够时等待（日志 `waiting for recovery source`），本周期直接跳过，不计为失败。
- `LOST` 下若 `recovery.automatic_when_lost=false`，只在手动调用 `/backend_node/relocalize` 后才尝试。
- 前端协调重置进行中时暂停。

### 4.2 候选来源（责任链，按优先级）

开机初始化（`INITIALIZING`）和局中恢复（`SUSPECT` / `LOST`）走不同的分支（`recovery_matcher.cpp`）。

**开机初始化**（不用 UWB）：

1. **启动提示** `startup_hint`（只在第 0 代初始化时，需 `enabled: true`）：在给定位姿附近验证。`force_until_matched: true` 时只用它，直到成功。搜索半径用 `recovery.initial_search_radius`（2 m）。
2. **Scan Context 描述子**：在里程计推算位置周围 `recovery.initial_search_radius`（2 m）内找。未配 `startup_hint` 时 `map ← odom` 为单位阵，推算位置就是地图原点（建图起点），所以开机位置必须在建图起点 2 m 内。`global_fallback_candidates: 0` 时附近找不到也不补全局候选，继续等待。描述子距离上限 `initial_max_descriptor_distance`（0.8）。

**局中恢复**（`SUSPECT` / `LOST`，只用 UWB）：

- 需要 `uwb.enabled: true`、已收到己方颜色（`/uwb_pose/team`，选用 `uwb.zero_pose_red` 或 `zero_pose_blue` 作为固定 `T_map_uwb`），且有新鲜的 `/uwb_pose`：按运动时刻（消息时间 + `uwb.time_offset_seconds`）不早于当前帧 `uwb.max_age`（1.2 s），并能在 `uwb.odom_sync_tolerance`（0.25 s）内插值到里程计。
- 满足时，天线点换算到 map，在 `uwb.search_radius`（2 m）内召回最多 `uwb.candidates`（8）个先验关键帧。同代次 `SUSPECT` 且存在可信位姿缓存时，先尝试最后可信全局变换与连续里程计前推得到的车体航向，失败再全圈扫描；`LOST` 或跨代次时直接全圈扫描。不再用描述子分支。
- 不满足时不做重定位：结果为推迟（`waiting for fresh UWB`），不计失败，状态保持 `SUSPECT`/`LOST`。所以 `uwb.enabled: false`（真车当前配置）或没有 UWB 的场地（仿真 RMUL）上，进入 `SUSPECT` 后不会自动恢复。
- UWB 不参与跟踪和因子图，也不用于判断定位是否正常。

日志里 `recovered via Scan Context + scan-to-map` 是初始化描述子分支成功，`recovered via UWB + scan-to-map` 是局中恢复成功；附近没有先验关键帧时日志为 `no prior KFs within radius`。

**为什么不做全局搜索**：RMUC/RMUL 场地中心对称，全局搜索容易锁到对称位置；UWB 给出独立的位置，对称位置（RMUC 相距约 19 m）不会成为候选。代价：开机位置须在建图起点 2 m 内（真车建议按出生点配置 `startup_hint`），局中恢复依赖 UWB。

**UWB 参数**：`uwb.zero_pose_red` / `zero_pose_blue` 是裁判系统坐标系到地图包 `map` 的固定变换（x、y、yaw），与地图包绑定，换地图要重设；不再在线估计。`uwb.antenna_position_body` 为天线在 `base_link` 中的位置，按测量时刻从 TF 查外参换算杆臂。

### 4.3 航向假设

描述子分支的每个候选关键帧先用描述子给出的 yaw 作初值；不通过时按 `recovery.yaw_sweep_count`（6，即每 60°）扫一圈。日志中 `hypothesis=descriptor_yaw+120deg` 即来自扫描。UWB 位置本身不测航向；同代次 `SUSPECT` 有可信位姿缓存时，先验证连续预测的航向，失败再回退完整扫描。`LOST`、跨代次或无可信缓存时没有该航向初值，直接全圈扫描。所有分支沿用相同几何和校正门限，不硬性限制恢复航向。启动提示只试提示航向。每次最多验证 `max_candidates`（15）个候选，`parallel_candidates`（16）路并行，实际并行数不超过 CPU 硬件线程数。

### 4.4 几何验证

对每个假设，把源点云与候选附近 `target_keyframes`（11）个、半径 `target_radius`（8 m）内的先验关键帧组成的子图做点到面配准（参数在 `registration`）。通过条件：

| 条件 | 参数 | 默认 |
| --- | --- | --- |
| 收敛（在最终对应集上再解一步，步长不超过阈值） | `registration.convergence_translation` / `convergence_rotation_deg` | 0.005 m / 0.1°（真车）；0.02 m / 0.5°（仿真） |
| 残差 | `registration.max_rmse` | 0.25 m |
| 对应点比例 | `registration.min_correspondence_ratio` | 0.5 |
| 对应点数 | `registration.min_correspondences` | 80 |
| 水平覆盖扇区 | `registration.min_coverage_sectors` | 6 / 36 |
| 可观性 | 6 自由度满秩，协方差有效 | — |

### 4.5 UWB 一致性与连续确认

通过验证的位姿不会立即提交，满足下面两个条件才提交并回到 `NORMAL`：

1. **与 UWB 一致**（只对 UWB 分支）：用恢复解推算的天线位置与 UWB 读数的水平偏差 ≤ `uwb.max_offset`（0.6 m）。超出时丢弃该候选并清空待确认候选，不计失败（日志 `recovery candidate rejected: inconsistent with UWB`）。`0` 关闭。
2. **连续一致**：连续 `recovery.confirmations`（2）个候选一致。每个新候选与**上一个**和**第一个**候选的差都不超过 `confirm_translation`（0.3 m）和 `confirm_rotation_deg`（5°），防止沿候选链逐步漂移；不一致则从头计数（日志 `recovery candidate inconsistent with previous ... vs first ...; restarting confirmation`）。

提交时日志为 `recovery candidate accepted after N consistent candidates via <来源>`。`confirmations: 1` 即“首次匹配就提交”。

原来的运动门（`confirm_min_translation` / `confirm_min_rotation_deg` / `confirm_stationary_max`）已于 2026-11-08 删除：UWB 提供了独立位置，不再靠车移动来排除对称解。开机初始化不做 UWB 检查，只靠 2 m 搜索半径和连续确认。

### 4.6 歧义检验与恢复试用期

- **歧义检验**（只在 `SUSPECT`/`LOST`，初始化不检验）：本次验证中若有另一个通过的解，与最佳解位姿相差 > `recovery.ambiguity_translation`（1 m）或 > `ambiguity_rotation_deg`（30°），且残差 ≤ 最佳残差 × `ambiguity_rmse_ratio`（1.15），就推迟恢复，不计为失败（日志 `ambiguous result, deferring recovery`）。`ambiguity_rmse_ratio: 0` 关闭。注意：描述子航向初值验证通过后不再扫描其余航向，180° 对称解通常没有进入比较，所以这项检验拦不住对称解。
- **恢复试用期**：恢复提交后的 `recovery.probation_observations`（5）次全局观测内，一次严重冲突（第 3.1 节）即回 `SUSPECT`。用于发现锁错的恢复，避免修正上限把后续纠正挡掉、同时一直报可信。

### 4.7 失败与升级

- `SUSPECT` 下有新鲜 UWB 但没有任何候选通过（或有历史帧但都被累积门限排除）：失败计数 +1，达到 3 次进 `LOST`。没有新鲜 UWB、结果有歧义时只推迟；候选与 UWB 不一致、或通过但确认门还没满足时只丢弃或记下候选，都不计失败。
- 进入 `LOST` 时，后端会向前端发协调重置（`/zlio/reset_internal`）；之后 `LOST` 下每次重定位失败（等待 UWB 不算失败）都会再请求一次，由挂起事务和 `reset.min_gap`（10 s）限流。
- 重置完成后：有先验地图时状态保持 `LOST`，靠 UWB 引导的重定位恢复（UWB 位置附近 2 m）。没有先验地图时取决于里程计连续性标记（`odometry_connected_`）：前端输入中断走的 `notifyFrontendDiscontinuity` 会先把它清掉，所以无论前端做暖重置还是冷启动，后端都会 `forceLost`。手动 `/localization_reset` 不清这个标记：暖重置边界有效时用 `old_from_new` 衔接，否则按停车假设用最后可信位姿衔接，然后进 `INITIALIZING`，下一帧回到 `NORMAL`（`reset_coordinator.cpp`）。**所以建图模式（无先验地图）下前端一旦输入中断，后端会一直保持 LOST。**

### 4.8 手动操作

| 服务 | 作用 |
| --- | --- |
| `/backend_node/relocalize` | 只在 `LOST` 下有效：请求在下一有效帧做一次重定位（同样需要新鲜 UWB，只在 UWB 位置附近搜索） |
| `/localization_reset` | 前后端协调重置；**调用期间必须停车**（无法暖重置时后端假设车静止来衔接位姿） |

---

## 5. 常规跟踪与回环

### 5.1 定位模式：常规跟踪

- 每个新的运行关键帧最多做一次全局观测，两次观测至少隔 `tracking.correction_period`（0.5 s，且受维护周期 `common.maintenance_period` 限制）。源点云取该关键帧及之前 ≤5 帧，在离预测位置最近（`nearest_reference_radius` 7 m 内）的先验关键帧附近子图上做 scan-to-map。
- 参考关键帧切换带 `target_hysteresis_distance`（0.5 m）滞回，避免来回跳。新参考未产生可用观测时，若上一参考仍存在且在 `nearest_reference_radius` 范围内，再用相同几何、创新和校正门限重试上一参考；只有其通过才采用，否则保留原失败结果。
- 结果按 3.1 节分为接受 / 冲突 / 无效。被接受的观测作为先验因子进图（日志 `prior scan-to-map factor accepted` + `global correction committed`）。

### 5.2 建图模式：关键帧与回环

- 回环开关是 `loop.enabled`，但加载了先验地图或描述子不可用时，`resolveConfig()`（`backend/src/config.cpp`）会把它强制关闭并打印 `configuration override`，所以实际只在建图模式运行。
- 关键帧：默认只按时间 `trajectory.keyframe.max_interval`（0.5 s）触发，距离/角度触发关闭（-1）。
- 回环检索（`loop.*`）：用 Scan Context 在已优化位姿 `candidate_max_distance`（7 m）内找历史关键帧，两次回环之间至少隔 `cooldown_keyframes`（5）帧。
- 回环验收：
  - 几何：相对平移 ≤ 3 m、高度 ≤ 0.5 m、roll/pitch ≤ 10°、与描述子 yaw 差 ≤ 60°、重叠率 ≥ 0.45（`loop.validation`）；
  - 一致性：PCM 最大团检查（`loop.pcm`，1 m / 15°）；
  - 统计：加入前的 NIS 检验。
- 接受后写入因子图并重新优化，日志 `loop scan-to-map accepted ... delta_t=..m`。

仿真实测（2 圈）：回环 18 次全部接受，单次修正 0.6–4 cm，轨迹 RMSE 约 2 cm。

---

## 6. 主要参数速查

只列常调的，括号内为 `bringup/config/<platform>/backend.yaml` 中的当前值。完整列表见 `backend/docs/parameters.md`。

### 6.1 坐标与外参 `common`

| 参数 | 说明 |
| --- | --- |
| `odom_frame` | 前端里程计帧；bringup 设为 `lio_odom` |
| `sensor_frame` | 前端位姿/点云所在帧，与前端 `frames.imu` 一致（仿真 `mid360_link`，真车 `mid360_imu_link`） |
| `body_frame` | 车体帧（`base_link`）。传感器到车体外参不在 yaml 中：按消息时间戳从 TF（URDF）查询 `sensor_frame ← body_frame`，云台 `yaw_joint` 变化被跟踪（位姿输出）。重新标定（`body_calibration.launch.py`）后把结果写入 URDF |
| `scan_context.lidar_height` | 雷达离地高度（0.2 m），影响描述子 |
| `descriptor_type` | `scan_context`（默认）或 `gprobe` |

### 6.2 地图 `prior_map` / `trajectory`

| 参数 | 说明 |
| --- | --- |
| `prior_map.enabled` / `directory` | 仅 launch：由 `mode`、`prior_map`（bringup 另有 `map_dir` 兜底）提供，不在 yaml 中 |
| `trajectory.save_directory` | 仅 launch：由 `map_dir`（bringup）或 `save_dir`（backend.launch）提供 |
| `trajectory.save_on_shutdown` | 正常退出时自动保存（false） |
| `trajectory.map_voxel_resolution` | 保存的 `map.pcd` 体素（0.15 m） |

### 6.3 配准 `registration`（跟踪、重定位、回环共用）

| 参数 | 默认 | 调大/调小的影响 |
| --- | --- | --- |
| `voxel_resolution` | 0.4 m | 越小越精细但越慢 |
| `max_iterations` | 16（仿真 30） | 不收敛时会跑满 |
| `convergence_translation` / `convergence_rotation_deg` | 0.005 m / 0.1°（仿真 0.02 m / 0.5°） | 收敛判据：最终一步的步长上限；不收敛的跟踪观测判为“模型无效”，一次进 `SUSPECT` |
| `max_correspondence_distance` | 1.0 m | 越大初值容忍越大、误匹配越多 |
| `max_rmse` | 0.25 m | 越小越严格 |
| `min_correspondence_ratio` | 0.5 | 越大越严格，地图覆盖不全时容易失败 |
| `degeneracy_condition_threshold` | 10 | Schur 退化判定阈值（日志中的 `schur_t`/`schur_r`/`weak=`） |

### 6.4 跟踪 `tracking` 与一致性 `innovation`

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `tracking.correction_period` | 0.5 s | 常规全局观测周期 |
| `tracking.max_translation_correction` / `max_rotation_correction_deg` | 1.5 m / 15° | 单次校正上限，超出视为冲突 |
| `tracking.suspicious_linear_speed` / `suspicious_angular_speed` | 10 m/s / 30 rad/s | 速度异常直接 `SUSPECT` |
| `innovation.rejection_probability` | 0.001 | NIS 检验阈值，越大越严 |
| `innovation.conflicts_before_suspect` | 7 | 连续冲突多少次进 `SUSPECT` |
| `innovation.severe_rejection_probability` / `severe_min_support_ratio` | 1e-8 / 0.6 | 严重冲突判定（第 3.1 节） |
| `innovation.severe_conflicts_before_suspect` | 2 | 累计多少次严重冲突直接进 `SUSPECT` |

### 6.5 重定位 `recovery`

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `period` | 0.2 s | 尝试间隔 |
| `initial_search_radius` | 2 m | 开机初始化（启动提示、描述子）的搜索半径（第 4.2 节） |
| `global_fallback_candidates` | 0 | 初始化时的全局补充候选；0 不做全局重定位 |
| `descriptor_distance_limit` / `initial_max_descriptor_distance` | 0.8 | 描述子门限（描述子只在初始化时用，实际生效的是后者） |
| `yaw_sweep_count` | 6 | 航向扫描档数 |
| `max_candidates` / `parallel_candidates` | 15 / 16 | 每次验证数量 |
| `target_keyframes` / `target_radius` | 11 / 8 m | 验证用子图 |
| `suspect_failures_before_lost` | 3 | `SUSPECT` → `LOST` |
| `automatic_when_lost` | true | LOST 时是否自动重定位 |
| `confirmations` / `confirm_translation` / `confirm_rotation_deg` | 2 / 0.3 m / 5° | 连续确认（第 4.5 节） |
| `probation_observations` | 5 | 恢复试用期（第 4.6 节） |
| `ambiguity_rmse_ratio` / `ambiguity_translation` / `ambiguity_rotation_deg` | 1.15 / 1 m / 30° | 歧义检验（第 4.6 节） |
| `source.min_frames` / `max_frames` / `max_time` / `max_translation` | 2 / 5 / 0.5 s / 1.5 m | 源点云累积 |

### 6.5a 局中恢复 UWB `uwb`

| 参数 | 仿真 / 真车 | 说明 |
| --- | --- | --- |
| `enabled` | true / false | 是否订阅 `/uwb_pose`、`/uwb_pose/team`；关闭或没有新鲜 UWB 时 `SUSPECT`/`LOST` 不重定位、只等待 |
| `zero_pose_red` / `zero_pose_blue` | 仿真 (23.42, 8.27, 180.15°)；真车未标定 | 裁判坐标系到 `map` 的固定变换，按己方颜色选用；与地图包绑定 |
| `time_offset_seconds` | -0.2 / -0.15 | 运动时刻 = 消息时间 + 此值 |
| `max_age` / `odom_sync_tolerance` | 1.2 / 0.25 s | UWB 读数的最大年龄（1 Hz 时须大于 1 s）、与里程计插值的最大时间差 |
| `search_radius` / `candidates` | 2 m / 8 | 围绕 UWB 位置召回关键帧的半径与数量 |
| `max_offset` | 0.6 m | 恢复解与 UWB 的最大水平偏差（第 4.5 节） |
| `antenna_position_body` | (-0.245, 0, 0) | 天线在 `base_link` 中的位置 |

### 6.6 回环 `loop`

| 参数 | 默认 |
| --- | --- |
| `enabled` | true（有先验地图或无描述子时由 `resolveConfig()` 强制关闭，所以只在建图模式生效） |
| `candidate_max_distance` | 7 m |
| `cooldown_keyframes` | 5 |
| `validation.min_overlap_ratio` | 0.45 |
| `validation.max_translation` | 3 m |
| `pcm.translation_threshold` / `rotation_threshold_deg` | 1 m / 15° |

### 6.7 其他

| 参数 | 说明 |
| --- | --- |
| `startup_hint.*` | 已知出生点时可开启，给初始化一个位姿先验（出生点固定的比赛场景很有用） |
| `output.world_cloud.*` | 世界点云开关、频率、不可信时是否发布 |
| `visualization.*` | `rviz:=true` 时 launch 会自动打开 |

---

## 7. 已知问题与建议（仿真测试结论）

测试方法与数据见 `tools/sim_tests/README.md`。

### 7.1 地图朝向覆盖不足 → 重定位乱跳（已确认）

MID-360 侧倾安装、视场不对称。建图时车头朝向不变，地图里每个位置就只有一个方向的点云；之后车一转身，配准支撑率掉到 0.4 左右，描述子给不出可靠 yaw，系统会接受相差约 90° 的错误位姿。

| 建图方式 | 原地自转时位置误差 | yaw 误差中位数 |
| --- | --- | --- |
| 朝向始终不变 | 3–5.6 m | 17.8° |
| 每个路点原地转一圈 | **约 4 cm** | **0.31°** |

**建图时要在场地各处原地转身**，尤其是出生点、补给区、常停留区域。

### 7.2 转动时频繁误进 `SUSPECT`（未修）

即使地图覆盖完整，原地转 4 圈仍进入 `SUSPECT` 约 10 次，每次 0.4–1.2 s 后恢复。日志原因是 `prior scan-to-map cannot construct a pose likelihood`，但当时 rmse 0.05–0.09 m、支撑率 0.65–0.92，匹配本身是好的。

- 原因：配准收敛判据（`registration_solver.cpp`）要求在最终对应集上再解一步的步长 ≤ `registration.convergence_translation` 且 ≤ `convergence_rotation_deg`。当时用的是上游值 5 mm / 0.1°、`max_iterations` 16，转动时这些观测跑满迭代仍未满足，被判“无效”；而“无效”一次就进 `SUSPECT`（第 3.1 节）。
- 现状：仿真配置已放宽为 0.02 m / 0.5°、30 次迭代（见 `src/z-localization/README.md`），本文未复测；真车仍为上游值。
- 影响：期间 `pose_trusted=false`，按可信标记停车的导航会停下。输出位姿由里程计前推，不再停住（2026-11-09 前为冻结）。
- 候选修法：把单次无效当作“本周期无观测”、连续 N 次才进 `SUSPECT`；或放宽/改写收敛判据。
- 已试无效：限制累积窗口转角 `source.max_rotation_deg: 10`。

### 7.3 绑架与场地 180° 中心对称（2026-11-08 起改为 UWB 引导的局部搜索，见本节末）

RMUL 场地红蓝两半绕中心旋转 180° 相同，另有局部几何相似的关键帧。仿真绑架（`set_pose` 瞬移）排查出三处根因：

| 问题 | 原因 | 修法 |
| --- | --- | --- |
| 瞬移后约 4 s 才进 `SUSPECT`，期间报可信 | 冲突再严重也要连续 7 次 | 严重冲突累计 2 次即升级（第 3.1 节） |
| 恢复后仍偏几米却报可信（双雷达约 3.7 m） | 修正上限挡掉后续正确修正 | 恢复试用期（第 4.6 节） |
| 恢复锁到对称位姿或相似关键帧 | 只要求相邻候选一致，车不动时错误解也能连续一致 | 当时加了运动门（已删除）；现在用 UWB 先验与一致性检查（第 4.2、4.5 节），歧义检验只部分有效 |

仿真结果（2026-11-02，`fabbcc5`，`kidnapspin`，各 5 场，详见 `tools/sim_tests/README.md`）：

- 双雷达 5/5 恢复，`FALSE TRUST` 0–2.6 s，恢复后可信样本最大误差 ≤ 0.31 m（修复前 9.0 s / 3.68 m）。
- 单雷达只有 1/5 干净恢复，其余锁到约 99°/180° 的错误位姿，或恢复正确后行驶中又跑偏。
- 车不动时运动门不起作用：双雷达停在不可信状态等待，或停车兜底提交错误解。

试过但撤回：最佳候选补验航向翻转 180° 的解（单雷达略有改善，双雷达恢复后误差退回 4–7 m）；`confirmations` 改为 3（无改善）。

当时的待做（单雷达需要更强的区分手段）现在由 UWB 位置先验承担。

**2026-11-08 更新**：上面是远距离绑架（5–7 m）加全局搜索、运动门时的数据。现在局中恢复只在 UWB 位置 2 m 内搜索、不做全局重定位（第 4.2 节），对称位置（RMUC 相距约 19 m）不会成为候选；仿真绑架也改为离跑圈终点 1.5 m、航向 45°（`zt_driver.py`）。没有 UWB 时（RMUL、真车当前配置）保持 `SUSPECT`/`LOST`。这套实现尚未跑仿真回归。

### 7.4 其他注意事项

- 测定位时不开 Gazebo GUI；非交互 shell 需设置 `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`，否则雷达掉到约 3 Hz。
- `ros2 topic hz` 对大点云会严重低估频率，用 `tools/sim_tests/cloud_rate.py`。
- 调用 `/localization_reset` 时必须停车。

---

## 8. 排查流程

```bash
# 1. 状态与原因
ros2 topic echo /backend_node/localization_state
# 2. 输入链路
ros2 topic info /lidar/points          # 只能有 1 个发布者
ros2 topic hz /lidar_odometry          # ≈ 雷达帧率
# 3. TF
ros2 run tf2_ros tf2_echo map base_link
```

| 日志关键字 | 含义 | 看什么 |
| --- | --- | --- |
| `observation model invalid` | 单次配准无效 → `SUSPECT` | `rmse`、`ratio`、`weak=`；转动时频发属 7.2 |
| `correction conflict k/7` | 观测与预测冲突 | NIS、校正量；外参、地图是否过期 |
| `recovery candidate inconsistent` | 连续确认未通过 | 偏差是否约 90°/180°（地图覆盖、对称） |
| `local recovery failed k/3 in SUSPECT` | 局部重定位失败 | 是否在地图覆盖范围内、`uwb.zero_pose_*` 是否与地图包匹配 |
| `waiting for fresh UWB` | 局中恢复在等 UWB | `/uwb_pose` 有无数据、`/uwb_pose/team` 是否已发、`uwb.max_age` |
| `inconsistent with UWB` | 恢复解与 UWB 不一致被丢弃 | `offset` 大小；`zero_pose`、天线位置、`time_offset_seconds` |
| `front-end input discontinuity` | 前端输入中断 → `LOST` | IMU/雷达是否断流、是否有重复节点 |
| `loop scan-to-map accepted` | 回环接受（建图） | `delta_t` 是否异常大 |
| `Prior map does not exist` | 启动报错 | `map_dir` / `prior_map` 路径 |
