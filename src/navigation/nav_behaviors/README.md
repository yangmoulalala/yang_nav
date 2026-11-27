# nav_behaviors

Nav2 plugins for following paths through annotated, direction-constrained terrain. The package keeps one shared passage pipeline; terrain labels provide geometry policy and select a small set of motion profiles.

## Design

```text
ZoneRecovery (BT root)
  ├─ navigation: plan → ZonePathShaper → ZoneAwareController (MPPI + ZoneTracker)
  │               ZoneReplanPaused retains the shaped path during ALIGN/TRAVERSE/EXIT
  └─ recovery: clear → spin → back up → repeat, with physical/phase no-motion protection
```

The pure geometry and tracking code has no ROS dependencies. Nav2-specific lifecycle, costmap, TF, topic, and Behavior Tree integration stays in thin plugin adapters.

## Modules

`src/core` implements geometry and tracking, `src/plugins` contains the Nav2 adapters, and `src/bt` contains the behavior-tree nodes. Declarations live under `include/nav_behaviors`.

| Module | Responsibility |
| --- | --- |
| `zones_yaml` | Load the terrain package's zone geometry and traversal policy. |
| `zone_geometry` | Transform coordinates and group adjacent zones under their directional policy. |
| `path_shaper` | Detect zone crossings, shape smooth axis-aligned paths, and reject colliding variants. |
| `start_aware_planner` | Prepend a fixed-heading escape from an occupied/inflated start, then invoke Smac2D without modifying the map. |
| `overlap_escape` | Bounded, non-growing footprint overlap with the original obstacle component; unknown/new obstacles remain blocked. |
| `zone_path_handoff` | Bind a committed shaped path to its goal, preventing raw-path leakage and stale-goal execution. |
| `zone_tracker` | Track the shaped path, manage the shared approach/align/traverse/exit phases, and calculate velocity commands. |
| `zone_state` | Shared phase definitions, parsing and the stamped phase memory (`PhaseMemory`) used by the BT nodes. |
| `recovery_policy` | Pure recovery ladder: clear costmaps -> spin -> back up -> repeat, with optional finite retries and no motion inside a zone. |
| `zone_path_shaper` | Nav2 smoother adapter: TF start heading, costmap collision checks, and path conversion. |
| `zone_aware_controller` | Nav2 controller adapter: MPPI handoff, tracker commands, dynamic obstacle stop, mode/state and RViz output. |
| `zone_replan_paused` | BT condition that blocks path replacement while the tracker owns the crossing. |
| `zone_recovery` | BT control node that retains the goal through recovery failures, waits between attempts, and supports cancellation and replacement. |

## Recovery

The root of both `navigate_to_pose.xml` and `navigate_through_poses.xml` is `ZoneRecovery` with five children: the navigation subtree, clear
costmaps, spin, back up, give up. `recovery_policy` picks one child per navigation failure, so
retries escalate instead of repeating the same action; `attempt` resets in `halt()`, i.e. per goal.
Both shipped trees use `retries="-1"`: a valid navigation goal stays active until it succeeds,
is cancelled, or is replaced. Outside protected zones the clear/spin/back-up ladder repeats,
so a long blockage does not cause endless back-up commands. A nonnegative `retries` value
retains the bounded policy (`0` gives up immediately). Values below `-1` also fail immediately.
`retry_delay="1.0"` adds a nonblocking one-second pause after every recovery attempt, including
failed or safety-interrupted recoveries; the clear subtree also waits one second for sensor refresh.
Cancellation and new goals interrupt the pause immediately. Invalid goals rejected before the
behavior tree starts (for example, a goal-frame transform error) still need corrected goal input.

Before motion recovery, the tree checks robot TF against the configured zones with a 0.55 m
margin. Physical occupancy, missing/stale TF, or a recent ALIGN / TRAVERSE / EXIT phase inhibits
spin and back-up. The inhibition latches for the failure sequence, so phase expiry while stopped
cannot enable motion. A running motion recovery is interrupted if protection becomes active.
The tree keeps clearing, waiting and retrying navigation while motion recovery is inhibited;
a new goal interrupts the old subtree and resets its recovery ladder. Costmap/TF/localization
and collision-monitor checks remain in force: retaining the goal does not require nonzero velocity
while the route is unsafe. Both single-goal and through-poses trees use this protection.

## Zone policy

`terrain_export.py` writes one `zones.yaml` record per connected directional region. Its `type` is the semantic terrain label. The following fields describe traversal independently of the label:

| Field | Meaning |
| --- | --- |
| `centre`, `yaw_deg`, `size` | Oriented zone box; yaw is its travel axis (for slopes and steps, positive yaw points uphill). |
| `one_way_dir` | `0` shapes either direction; `1` or `-1` selects that signed axis for shaping/tracking. Opposite-direction descent currently stays with ordinary planning/MPPI. |
| `allow_flip` | Whether the shaper may choose either physical heading along the axis. The tracker retains the collision-checked path heading. |
| `recentre` | Whether the shaped path should move to the zone centerline. |
| `warning` | Ambiguous directions disable axis locking instead of choosing an arbitrary average. |

Defaults are exported explicitly: tunnels and undulations are bidirectional; slopes and steps select their annotated positive direction for the zone tracker; tunnels are centered while open terrain retains the planner's lateral line. Old zone files without policy fields receive the same defaults in the loader.

Touching open directional faces with different axes additionally require a centred uphill crossing, even when `recentre` is false. Their bounding rectangles can overlap along a physical seam (for example, a hexagonal fortress); keeping the grid path's lateral line there can put the wheels on two slopes. `groupZones` derives `seam_constrained` using a separating-axis test with a 5 cm boundary allowance and excludes parallel/opposing, distant and ambiguous faces. Faces stay separate. Shaping uses their central corridor and holds the exit heading until the full chassis clears the face. This policy does not change the terrain annotations or delegate downhill differently.

The semantic type selects tuning: `tunnel` uses the narrow-passage speed and footprint margins; other configured directional terrain uses the open-terrain (ramp) profile. Both use the same shaping and tracking algorithms. Add a new terrain label by exporting its policy and selecting it in the plugin `types` parameter; do not add another controller state machine.

## Nav2 parameters

Both the smoother and controller load the same `zones.yaml` and must use matching type lists and chain parameters. The shipped configuration enables:

```yaml
types: [tunnel, slope, step_l1, step_l2, step_high, fly_slope, undulation]
```

The controller's `zone.*` parameters provide two tuning profiles (`tunnel_*` and `ramp_*`), shared alignment gains, collision margins, and chaining thresholds. `/nav_zone_state` reports the current phase; `/nav_mode` applies the matching `cmd_gate` speed backstop.

## Occupied starts and exit replanning

An occupied start does not immediately abort navigation. The planner finds a translation-only
escape (default at most 1.5 m), and the controller executes it at at most 0.15 m/s before normal
tracking resumes. The smoother preserves and validates that prefix. Overlap area with the
original connected obstacle component may stay constant or shrink; unrelated/new obstacle cells,
unknown space and map bounds are never exempted. The controller freezes that component for the
whole episode. The live collision monitor and localization/observation gates remain enabled.
This handles map overlap, not a guarantee that a physically embedded chassis can move.
For single-goal localization navigation, if no route to the target can be planned but a safe
exit exists, the planner can return just the escape segment. Its endpoint is not the original
goal. If a static or live obstacle occupies the target, the planner tries reachable approach
points within 2 m with extra footprint clearance for stopping. The original goal remains
pending; when it clears, replanning can resume navigation to it. Map bounds and the hardware
collision monitor are not relaxed. Strict waypoint planning does not substitute waypoints.
An escape failure publishes `blocked` so the BT can replan; a newly committed shaped path
replaces the previous escape target and allowance. A changed goal clears tracker progress.

Outside directional zones, an XY-only platform with unobserved chassis heading holds still
if its rotating reference frame encounters a footprint overlap. It does not wait for the
reference yaw to stop after the overlap disappears. The center and full padded footprint must
remain clear across at least 1.0 s of new local-map updates and sensor observations (both at
most 0.50 s old). The connector back to the committed path and its local preview must be free
before normal collision-checked tracking resumes. Directional zones retain their escape policy.

A path starting just beyond a zone's far mouth still creates an EXIT crossing. The tracker
holds the body heading while the complete padded rear clears the zone, including after a cold
restart or replanning that removed every body sample.

Both BTs capture the planning target, keep raw and shaped paths separate, and commit only a
successfully shaped path for the same target. A new goal waits for its own committed path.

## Tests and build

The smoother and zone controller check the complete padded convex footprint, including occupied
cells inside its perimeter. Motion checks interpolate translation and rotation at half-cell steps;
unknown and out-of-map space are rejected. `collision_tolerance` must be zero. A required shaping
failure propagates to Nav2 instead of handing an unshaped path to the tracker. Tracking also checks
the measured and commanded braking trajectories, including lateral motion, rotation and reversing.

Core tests cover path shaping, direction policies, tracking, state transitions, footprint collision,
closed-loop zone traversals, and recovery inside a real behaviour tree (`test_zone_recovery`).
The integration tools in `tools/nav_regress` run real Nav2 nodes with deterministic test inputs.
The kinematic checks use a fixed geometry sample in `test/fixtures/rmuc_zones.yaml`. Field regressions read the operator's selected terrain directory. See the [regression tools](../../../tools/nav_regress/README.md).


### 真车与仿真共用仅平移控制

共享 `nav2.yaml` 为 ZoneAwareController 选择 `TranslationMPPIController`：采样和优化 X/Y，禁止优化主动旋转，预测时所有候选采用同一个实测角速度，以保留被动航向变化对轨迹与轮廓的影响。普通控制输出 angular.z 恒为 0。使用闭环里程计，不支持 open_loop=true。

两种环境均使用 `TranslationGoalChecker`，要求当前 XY 距离 ≤0.15 m 且实测平移合速度 ≤0.03 m/s，不锁存经过容差区的状态，不依赖目标 yaw；自动恢复关闭 Spin。方向约束地形在 body_heading_available=false 时停止并报告底盘航向不可观测；即使云台刚好对齐也不能放行，不会把普通角速度伪装成动作角度。自动触发和完成动作需要另行实现 MCU 动作反馈，当前不自动发送动作。

`KeepOccupiedGoalPending` 只在目标变更，或已到替代终点等待时收到有效的新终点/可达路径，才重启子节点。正在执行的正常重规划交由 FollowPath 更新，5 cm 栅格抖动不会取消跟踪。

控制频率 20 Hz 沿用此前真车调试配置，与 MPPI 的 0.05 s 模型步长匹配，使优化序列每周期前移；预测时域 1 s、500 个候选。平滑器和最终安全门仍为 100 Hz。Jazzy MPPI 允许控制周期小于模型步长（此时不在每次控制时前移序列），因此提高到 100 Hz 并非接口错误，但应先测量计算耗时和掉帧。

共享 `nav2.yaml` 中，普通地形每轴允许 ±2 m/s，最终安全门按合速度限幅至 2 m/s；百分比地形限速的参考同样为 2 m/s。真车另叠加 `nav2_real.yaml`，最终普通合速度上限为 1.50 m/s，路径跟踪参考上限为 1.00 m/s。`TranslationVelocityCritic` 用当前位置投影后的 2 m 前瞻确定参考速度大小，到终点附近降速。真车另取前方 0.10 m 确定方向，优先纠正横向误差，并对前 0.60 s 的预测速度评分。仿真保留原方向前瞻和两步评分。

MPPI 保留整段预测中的碰撞与约束评分，ZoneAwareController 和最终碰撞监视器继续检查制动轨迹。PathAlign 从第 1 个路径点启用、逐点评估、权重 40；PathFollow 前瞻 3 点、权重 10，在距终点 0.25 m 内退出；GoalCritic 在 0.30 m 内接管。上述碰撞与路径评分配置由真车与仿真共用。

默认真车配置在当前航向所定义的固定基底中优化平移速度，再将各预测步换算到该步的旋转参考系。100 Hz 平滑器也在 `map` 中限制平移加速度，输出前才按最新里程计航向换回 `gimbal_nav`；缺少新鲜航向时立即清零。这样被动旋转不会消耗平移加速度额度。速度上限、碰撞监视器和最终安全门保持不变。真车配置启用 `FollowPath.fixed_frame_translation` 时使用固定坐标系平滑器，包括 WebUI 保存的调参文件；仿真和未启用该选项的配置继续使用 Nav2 平滑器。

真车按当前局部路径重新初始化候选序列，而不是沿用重规划前的优化结果。候选加减速优先从最近一次有效的最终 `/cmd_vel` 起步；没有新鲜指令时从实测值反推。预测的第一步始终使用实测运动，候选随后仍经运动模型与轮廓碰撞评分。若输出预计在 0.35 s 后距路径超过 0.07 m，且比当前距离增加超过 5 mm，则拒绝该指令，进入原有零速与失败处理并保留地形限速。它允许向路径返回，不把未经优化的参考速度直接发给底盘；0.07 m 是护栏触发条件，不是实车偏差保证。


真车 X/Y 在 `gimbal_nav` 云台参考系内。代价地图、障碍云、BT、恢复与控制器共用该帧；
不能直接使用 CAD `yaw_link` 的轴向。未知底盘 yaw 时 footprint 覆盖原矩形的全部相对转角。
插件的 `body_heading_available` 默认 true，共享导航配置显式设为 false；接入反馈前不允许以云台角度判定底盘定向通过。

普通 XY 模式下，足印占用且实测角速度超过 0.05 rad/s 时先发零速等待。占用持续 1.0 s 后，仅在地图和观测仍新鲜时尝试原有低速、只平移、不得增加接触的脱困；运动状态或走廊不允许安全脱困时报告 `NoValidControl`，交给导航恢复。若足印已清空，仍须确认 1.0 s 连续的新鲜清空观测，并检查重新接入的路径前段；确认无法完成时也报告失败，不无限等待。


真车启用 `FollowPath.response_feedback`：对比最终 `/cmd_vel` 与定位里程计，按世界系对齐近期稳定命令窗口（0.1～0.4 s），仅在速度至少 0.08 m/s、命令变化不超过 0.035 m/s 时更新平面增益/旋转模型。只接受 0.7～1.3 倍增益和 ±30° 以内的观测，滤波时间常数 0.35 s；停机、断开底盘、急变命令和异常响应均不参与辨识。模型仅保留当前进程内，控制器 reset 清除，不写入安装偏角或关节值。预测时对所有候选 XY 速度应用当前模型，使路径评分与 MPPI 轮廓碰撞评分看到相同响应；不会在优化器输出之后附加未经评分的速度旋转。该模型只覆盖有限范围的近似平面增益/旋转，不能识别全部 MCU 动力学，实物轴向与反馈仍需确认。

控制器的输出 `cmd_vel` 重映射到 `cmd_vel_nav`；插件的反馈输入 `~/executed_velocity` 单独重映射到 `/cmd_vel`，避免重映射后读回自己的原始指令。该反馈是安全门后的最终指令，不是轮速。启用固定坐标系平移和路径偏离检查时，路径速度参考至少前瞻当前参考速度在 0.35 s 内走过的弧长，并限制速度以免越过前瞻点。MPPI 仍对候选轨迹做碰撞评分，最后的偏离判据、限速、碰撞监控和安全门不变。

在弯道上，用前瞻路径的转角估计局部曲率，并按控制器与平滑器较小的 XY 加减速能力降低参考速度。直线不受此项影响。这个限制仅作用于优化器的速度参考，不修改底盘速度上限。

普通固定坐标系 XY 模式下，若 MPPI 求解成功但输出未通过末级偏离检查，仅保留它在纠偏方向上的正向速度分量，不增加速度模长。替代指令必须再次通过原偏离检查与当前轴向限速，并在最新局部图上确认整个制动圆盘均通行。圆盘覆盖任意朝向的带 padding 车体、反应距离与制动距离；按实测、上一条最终指令、候选指令三者的较大速度计算。数据过期、未知区域、膨胀内切区、障碍、地图边界或没有前进分量时仍拒绝指令，绝不接管优化器自身的碰撞或求解失败。
