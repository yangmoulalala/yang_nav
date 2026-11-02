# ZLIO 定位后端使用与机制说明

本文说明 `src/z-localization/backend`（节点 `backend_node`）当前的工作方式：怎么启动、有哪些状态、什么时候触发重定位、主要参数的含义，以及在仿真测试中已知的问题。

- 话题总表见 [topics_and_usage.md](topics_and_usage.md)。
- 全部参数（含离线工具）见 `src/z-localization/backend/docs/parameters.md`。
- 仿真回归测试见 `tools/sim_tests/README.md`。

内容对应分支 `kidnap-recovery-fix`（在 `single-lidar` 基础上增加严重冲突、恢复试用期、歧义检验和恢复运动门）。

---

## 1. 后端做什么

前端 ZLIO 只给出局部里程计（`lio_odom →` 前端 `frames.imu`，仿真 `mid360_link`、真车 `mid360_imu_link`），会慢慢漂移。后端负责把它放进全局地图坐标系 `map`：

| 模式 | 做什么 | 输出 |
| --- | --- | --- |
| 建图 `mapping` | 用关键帧建位姿图，Scan Context 检索回环，iSAM2 优化；可保存地图包 | `map → lio_odom`，`map` 原点 = 前端起点 |
| 定位 `localization` | 加载不可变的先验地图包，周期性做 scan-to-map 配准修正漂移；丢失时重定位 | `map → lio_odom`，`map` = 先验地图坐标系 |

输入只有三个话题：`/frontend/frame`（去畸变点云 + 位姿）、`/frontend/odometry`（高频预测）、`/frontend/session`（前端身份、代次、中断/失败通知）。可选 UWB 输入 `/uwb_pose`。

主要输出：

| 输出 | 说明 |
| --- | --- |
| TF `map → lio_odom` | 与 ZLIO 发的 `lio_odom → base_link` 组成 `map → base_link`，导航首选 |
| `/backend_node/localization_pose` | map 系车体位姿 + 协方差 |
| `/localization/odometry` | map 系传感器（`common.sensor_frame`）位姿，高频 |
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

前端已在运行时，可以只启动/重启 backend 包的 launch（调参时只重启它）。用 bringup 的参数文件即可与全套启动一致：

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
| `SUSPECT` | 怀疑出错，正在局部重定位 | false | 冻结在最后可信位姿；`/localization/odometry` 速度清零、协方差放大 |
| `LOST` | 丢失，等待全局重定位 | false | 同上 |

**导航必须订阅 `localization_state`，`pose_trusted=false` 时停车或降级。** 冻结期间车如果还在动，输出位姿与真实位姿的误差会随时间增大（实测原地 0.6 rad/s 自转时每 0.2 s 增加约 8°）。

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
   估计器失败 / 前端输入中断 ───────────────▶ LOST ──────────┘ 全局重定位成功
```

| 事件 | 条件 | 结果 |
| --- | --- | --- |
| 全局观测被接受 | 配准有效且 NIS 检验通过（p ≥ `innovation.rejection_probability`=0.001） | 清零计数，回到 `NORMAL` |
| 观测冲突 | 配准有效但与预测不一致（NIS 检验失败，或校正超过 `tracking.max_translation_correction` 1.5 m / `max_rotation_correction_deg` 15°） | 计数 +1；未到 `innovation.conflicts_before_suspect`（7）为 `OBS_DEGRADED`，到了进 `SUSPECT` |
| 严重冲突 | 冲突且配准对应比例 ≥ `innovation.severe_min_support_ratio`（0.6），并触发校正上限或 NIS p < `severe_rejection_probability`（1e-8）。配准可靠而修正极端，典型于绑架或错误恢复 | 连续冲突中累计 `severe_conflicts_before_suspect`（2）次即进 `SUSPECT`，不必等 7 次（日志 `(severe 2/2)`） |
| 恢复试用期内严重冲突 | 恢复提交后 `recovery.probation_observations`（5）次全局观测内 | 一次即进 `SUSPECT`（日志 `during recovery probation`） |
| **观测模型无效** | 配准不收敛、信息矩阵无效、或 rmse 超出局部模型范围 | **一次就进 `SUSPECT`**（不计数） |
| 速度异常 | 线速度 > `tracking.suspicious_linear_speed`（10 m/s）或角速度 > `suspicious_angular_speed`（30 rad/s） | 进 `SUSPECT` |
| `SUSPECT` 下重定位失败 | 连续 `recovery.suspect_failures_before_lost`（3）次 | 进 `LOST` |
| 估计器失败 / 前端输入中断 | 例如前端报告 `reset_required` 或 `failed` | 进 `LOST` |
| 重定位成功 | 见第 4 节 | 回 `NORMAL` |

“观测无效”（不能构造似然）与“观测冲突”（能构造，但和预测不符）是两条不同的路径：前者没有缓冲，这是第 7.2 节问题的来源。

---

## 4. 重定位触发机制

代码：`backend/src/localization_recovery.cpp`、`recovery_matcher.cpp`、`relocalization_*.cpp`。

### 4.1 何时运行

- 只要状态是 `INITIALIZING`、`SUSPECT` 或 `LOST`，后端就自动进入恢复流程；`NORMAL`/`OBS_DEGRADED` 下不运行重定位，只做常规跟踪。
- 调度间隔 `recovery.period`（0.2 s），且必须有新的同步帧。
- 源点云：累积最近 `recovery.source.min_frames`～`max_frames`（2～5）帧、时间跨度 ≤ 0.5 s、平移 ≤ 1.5 m。帧数不够时等待（日志 `waiting for recovery source`）。
- `LOST` 下若 `recovery.automatic_when_lost=false`，只在手动调用 `/backend_node/relocalize` 后才尝试。
- 前端协调重置进行中时暂停。

### 4.2 候选来源（责任链，按优先级）

1. **启动提示** `startup_hint`（只在第 0 代初始化时，需 `enabled: true`）：在给定位姿附近验证。`force_until_matched: true` 时只用它，直到成功。
2. **UWB 位置先验**（`uwb.enabled`）：在 UWB 位置 `uwb.recovery_search_radius`（2 m）内召回最多 `recovery_candidates`（8）个关键帧。
3. **Scan Context 描述子**：
   - `SUSPECT`：只在最后可信位置周围 `recovery.suspect_search_radius`（30 m）内找，不做全局补充；
   - `INITIALIZING` / `LOST`：在预测位置周围 `lost_search_radius`（35 m）内找，失败再补 `global_fallback_candidates`（5）个全局候选；
   - 描述子距离上限：初始化 `initial_max_descriptor_distance`，恢复 `descriptor_distance_limit`（都是 0.8）。

日志里 `recovered via Scan Context + scan-to-map` 表示第 3 条路径成功。

### 4.3 航向假设

每个候选关键帧先用描述子给出的 yaw 作初值；不通过时按 `recovery.yaw_sweep_count`（6，即每 60°）扫一圈。日志中 `hypothesis=descriptor_yaw+120deg` 即来自扫描。每次最多验证 `max_candidates`（15）个候选，`parallel_candidates`（16）路并行。

### 4.4 几何验证

对每个假设，把源点云与候选附近 `target_keyframes`（11）个、半径 `target_radius`（8 m）内的先验关键帧组成的子图做点到面配准（参数在 `registration`）。通过条件：

| 条件 | 参数 | 默认 |
| --- | --- | --- |
| 收敛（最终一步 ≤ 5 mm 且 ≤ 0.1°，代码常量） | — | — |
| 残差 | `registration.max_rmse` | 0.25 m |
| 对应点比例 | `registration.min_correspondence_ratio` | 0.5 |
| 对应点数 | `registration.min_correspondences` | 80 |
| 水平覆盖扇区 | `registration.min_coverage_sectors` | 6 / 36 |
| 可观性 | 6 自由度满秩，协方差有效 | — |

### 4.5 连续确认与运动门

通过验证的位姿不会立即提交，而是先换算成 `map ← odom` 变换，满足下面全部条件才提交并回到 `NORMAL`：

1. **连续一致**：连续 `recovery.confirmations`（2）个候选一致。每个新候选与**上一个**和**第一个**候选的差都不超过 `confirm_translation`（0.3 m）和 `confirm_rotation_deg`（5°），防止沿候选链逐步漂移；不一致则从头计数（日志 `recovery candidate inconsistent with previous ... vs first ...; restarting confirmation`）。
2. **运动门**：自第一个候选起，里程计平移 ≥ `confirm_min_translation`（0.3 m）或转角 ≥ `confirm_min_rotation_deg`（30°）。车静止时，对称或相似场景的错误解会连续自洽；视野变化后错误解才会和里程计对不上。两项都设 0 关闭运动门。
3. **停车兜底**：运动门开启但车一直不动时，一致候选累计 `confirm_stationary_max`（15）次也提交（日志带 `(stationary limit)`）；设 0 则必须移动。仿真中停车兜底有时会提交错误解。

提交时日志为 `recovery candidate accepted after N consistent candidates, motion=..m/..deg`。`confirmations: 1` 且关闭运动门即恢复“首次匹配就提交”。

**建议**：`SUSPECT`/`LOST` 期间让车或云台（`yaw_link`）慢转，给运动门提供视角变化。仿真用 `zt_driver.py` 的 `kidnapspin` 标志模拟。

### 4.6 歧义检验与恢复试用期

- **歧义检验**（只在 `SUSPECT`/`LOST`，初始化不检验）：本次验证中若有另一个通过的解，与最佳解位姿相差 > `recovery.ambiguity_translation`（1 m）或 > `ambiguity_rotation_deg`（30°），且残差 ≤ 最佳残差 × `ambiguity_rmse_ratio`（1.15），就推迟恢复，不计为失败（日志 `ambiguous result, deferring recovery`）。`ambiguity_rmse_ratio: 0` 关闭。注意：描述子航向初值验证通过后不再扫描其余航向，180° 对称解通常没有进入比较，所以这项检验拦不住对称解。
- **恢复试用期**：恢复提交后的 `recovery.probation_observations`（5）次全局观测内，一次严重冲突（第 3.1 节）即回 `SUSPECT`。用于发现锁错的恢复，避免修正上限把后续纠正挡掉、同时一直报可信。

### 4.7 失败与升级

- `SUSPECT` 下没有任何候选通过：失败计数 +1，达到 3 次进 `LOST`。
- 进入 `LOST` 时，后端会向前端发协调重置（`/zlio/reset_internal`）。冷启动后前端里程计从零开始，必须再次通过先验地图或 UWB 确定全局位姿；**建图模式没有先验地图，一旦 LOST 会一直保持 LOST。**

### 4.8 手动操作

| 服务 | 作用 |
| --- | --- |
| `/backend_node/relocalize` | 只在 `LOST` 下有效：请求在下一有效帧做一次全局重定位 |
| `/localization_reset` | 前后端协调重置；**调用期间必须停车**（无法暖重置时后端假设车静止来衔接位姿） |

---

## 5. 常规跟踪与回环

### 5.1 定位模式：常规跟踪

- 每 `tracking.correction_period`（0.5 s）取最近 ≤5 帧点云，在离当前位置最近（`nearest_reference_radius` 7 m 内）的先验关键帧附近子图上做 scan-to-map。
- 参考关键帧切换带 `target_hysteresis_distance`（0.5 m）滞回，避免来回跳。
- 结果按 3.1 节分为接受 / 冲突 / 无效。被接受的观测作为先验因子进图（日志 `prior scan-to-map factor accepted` + `global correction committed`）。

### 5.2 建图模式：关键帧与回环

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
| `body_frame` | 车体帧（`base_link`）。传感器到车体外参不在 yaml 中：按消息时间戳从 TF（URDF）查询 `sensor_frame ← body_frame`，云台 `yaw_joint` 变化被跟踪（位姿输出与 UWB 杆臂）。重新标定（`body_calibration.launch.py`）后把结果写入 URDF |
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
| `max_iterations` | 16 | 不收敛时会跑满 |
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
| `suspect_search_radius` / `lost_search_radius` | 30 / 35 m | 候选搜索半径 |
| `global_fallback_candidates` | 5 | LOST/初始化时的全局补充候选 |
| `descriptor_distance_limit` / `initial_max_descriptor_distance` | 0.8 | 描述子门限 |
| `yaw_sweep_count` | 6 | 航向扫描档数 |
| `max_candidates` / `parallel_candidates` | 15 / 16 | 每次验证数量 |
| `target_keyframes` / `target_radius` | 11 / 8 m | 验证用子图 |
| `suspect_failures_before_lost` | 3 | `SUSPECT` → `LOST` |
| `automatic_when_lost` | true | LOST 时是否自动全局重定位 |
| `confirmations` / `confirm_translation` / `confirm_rotation_deg` | 2 / 0.3 m / 5° | 连续确认（第 4.5 节） |
| `confirm_min_translation` / `confirm_min_rotation_deg` / `confirm_stationary_max` | 0.3 m / 30° / 15 | 运动门与停车兜底（第 4.5 节） |
| `probation_observations` | 5 | 恢复试用期（第 4.6 节） |
| `ambiguity_rmse_ratio` / `ambiguity_translation` / `ambiguity_rotation_deg` | 1.15 / 1 m / 30° | 歧义检验（第 4.6 节） |
| `source.min_frames` / `max_frames` / `max_time` / `max_translation` | 2 / 5 / 0.5 s / 1.5 m | 源点云累积 |

### 6.6 回环 `loop`

| 参数 | 默认 |
| --- | --- |
| `enabled` | true（仅建图模式生效） |
| `candidate_max_distance` | 7 m |
| `cooldown_keyframes` | 5 |
| `validation.min_overlap_ratio` | 0.45 |
| `validation.max_translation` | 3 m |
| `pcm.translation_threshold` / `rotation_threshold_deg` | 1 m / 15° |

### 6.7 其他

| 参数 | 说明 |
| --- | --- |
| `uwb.*` | 默认配置里启用，bringup 已关闭。接入时配置话题、天线位置、零位 |
| `startup_hint.*` | 已知出生点时可开启，给初始化一个位姿先验（出生点固定的比赛场景很有用） |
| `output.world_cloud.*` | 世界点云开关、频率、不可信时是否发布 |
| `visualization.*` | `rviz:=true` 时 launch 会自动打开 |
| `services.*` | 服务名 |

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

- 原因：配准收敛判据要求最终一步 ≤ 5 mm 且 ≤ 0.1°（`registration_solver.cpp`），转动时这些观测都跑满 16 次迭代仍未满足，被判“无效”；而“无效”一次就进 `SUSPECT`（第 3.1 节）。
- 影响：期间输出冻结，导航拿到的是停住的位姿。
- 候选修法：把单次无效当作“本周期无观测”、连续 N 次才进 `SUSPECT`；或放宽/改写收敛判据。
- 已试无效：限制累积窗口转角 `source.max_rotation_deg: 10`。

### 7.3 绑架与场地 180° 中心对称（双雷达已缓解，单雷达未修）

RMUL 场地红蓝两半绕中心旋转 180° 相同，另有局部几何相似的关键帧。仿真绑架（`set_pose` 瞬移）排查出三处根因：

| 问题 | 原因 | 修法 |
| --- | --- | --- |
| 瞬移后约 4 s 才进 `SUSPECT`，期间报可信 | 冲突再严重也要连续 7 次 | 严重冲突累计 2 次即升级（第 3.1 节） |
| 恢复后仍偏几米却报可信（双雷达约 3.7 m） | 修正上限挡掉后续正确修正 | 恢复试用期（第 4.6 节） |
| 恢复锁到对称位姿或相似关键帧 | 只要求相邻候选一致，车不动时错误解也能连续一致 | 运动门（第 4.5 节），歧义检验只部分有效 |

仿真结果（2026-11-02，`fabbcc5`，`kidnapspin`，各 5 场，详见 `tools/sim_tests/README.md`）：

- 双雷达 5/5 恢复，`FALSE TRUST` 0–2.6 s，恢复后可信样本最大误差 ≤ 0.31 m（修复前 9.0 s / 3.68 m）。
- 单雷达只有 1/5 干净恢复，其余锁到约 99°/180° 的错误位姿，或恢复正确后行驶中又跑偏。
- 车不动时运动门不起作用：双雷达停在不可信状态等待，或停车兜底提交错误解。

试过但撤回：最佳候选补验航向翻转 180° 的解（单雷达略有改善，双雷达恢复后误差退回 4–7 m）；`confirmations` 改为 3（无改善）。

待做：单雷达需要更强的区分手段，例如并行保留多个假设、随车移动淘汰，或把运动门转角提高到 ≥ 90°；单雷达配置可考虑 `confirm_stationary_max: 0`。也可以用 UWB / `startup_hint` 提供区分信息。

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
| `local recovery failed k/3 in SUSPECT` | 局部重定位失败 | 是否在地图覆盖范围内 |
| `front-end input discontinuity` | 前端输入中断 → `LOST` | IMU/雷达是否断流、是否有重复节点 |
| `loop scan-to-map accepted` | 回环接受（建图） | `delta_t` 是否异常大 |
| `Prior map does not exist` | 启动报错 | `map_dir` / `prior_map` 路径 |
