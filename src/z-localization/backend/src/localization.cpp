/**
 * @file localization.cpp
 * @brief 协调同步输入、维护任务和唯一状态提交边界。
 */

#include "backend/localization.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <type_traits>
#include <utility>

#include "backend/logger.hpp"

namespace backend {
namespace {

/// 以原子门闩保证同一时刻只运行一个维护周期；析构时释放所持准入。
class SingleFlightGate {
public:
    /// 尝试原子获取维护周期的独占准入。
    explicit SingleFlightGate(std::atomic<bool>& flag) : flag_(flag) {
        bool expected = false;
        acquired_ = flag_.compare_exchange_strong(expected, true);
    }
    /// 仅在成功获取准入时释放原子门闩。
    ~SingleFlightGate() {
        if (acquired_) {
            flag_.store(false);
        }
    }
    /// 返回当前对象是否持有独占准入。
    explicit operator bool() const { return acquired_; }

    /// 禁止复制维护周期准入对象。
    SingleFlightGate(const SingleFlightGate&) = delete;
    /// 禁止复制独占准入对象，避免门闩所有权重复。
    SingleFlightGate& operator=(const SingleFlightGate&) = delete;

private:
    std::atomic<bool>& flag_;
    bool acquired_ = false;
};

}  // namespace

/// 绑定地图、图估计和外部端口，并解析当前有效能力。
Localization::Localization(KeyframeMap& trajectory, KeyframeMap& prior, PlaceDescriptor& descriptor,
                           Config requested, SteadyClockPort& clock, FrontendResetPort& reset,
                           StateSink& state, VisualizationSink& visualization)
    : Localization(trajectory, prior, descriptor,
                   resolveConfig(std::move(requested), descriptor.enabled()), clock, reset, state,
                   visualization) {
}
/// 按已解析的有效配置初始化图估计，并绑定地图和外部端口。
Localization::Localization(KeyframeMap& trajectory, KeyframeMap& prior_map,
                           PlaceDescriptor& descriptor, EffectiveConfig effective_config,
                           SteadyClockPort& clock, FrontendResetPort& reset_port,
                           StateSink& state_sink, VisualizationSink& visualization_sink)
    : trajectory_(trajectory),
      prior_map_(prior_map),
      descriptor_(descriptor),
      config_(std::move(effective_config.values)),
      clock_(clock),
      reset_port_(reset_port),
      state_sink_(state_sink),
      visualization_sink_(visualization_sink),
      graph_(config_.graph),
      target_cache_(32, config_.storage.submap_cache_bytes),
      frames_(config_.tracking.source, config_.recovery.source),
      poses_(config_.graph.odom_noise),
      tracking_({config_.tracking, config_.relocalization.point_to_plane_config,
                 config_.registration_debug_enabled},
                prior_map_, target_cache_),
      recovery_({config_.recovery, config_.relocalization, config_.registration_debug_enabled},
                prior_map_, descriptor_, target_cache_),
      loop_cycle_({config_.loop, config_.relocalization.point_to_plane_config}) {
    for (const ConfigOverride& override : effective_config.overrides) {
        BACKEND_WARN_STREAM("configuration override: " << override.field
                                                       << " requested=" << override.requested
                                                       << " -> effective=" << override.effective
                                                       << " (" << override.reason << ")");
    }
    if (config_.prior_map_enabled && !descriptor_.enabled() &&
        !config_.startup_hint.T_map_sensor.has_value()) {
        BACKEND_WARN_STREAM(
            "prior map has no startup hint or descriptor; the backend "
            "cannot obtain an initial global pose");
    }
    publishOutputViewLocked();
}

/// 丢弃失败估计事务并以最后已提交的可信输出建立恢复边界。
void Localization::isolateEstimatorFailureLocked(const std::string& reason) {
    const auto committed = std::atomic_load_explicit(&output_view_, std::memory_order_acquire);
    if (!committed)
        throw std::logic_error("estimator has no committed output to recover");

    // 恢复最后完整提交的输出，失效代次覆盖所有锁外任务和高频投影。
    poses_ = committed->poses;
    active_epoch_ = committed->epoch;
    odometry_connected_ = committed->odometry_connected;
    poses_.advanceRevision();
    state_.onEstimatorFailure(reason);
    estimator_failed_ = true;
    state_before_manual_reset_.reset();

    // 运行轨迹与描述子共同重新编号，先验地图保持独立生命周期。
    graph_.reset(config_.graph);
    trajectory_.clear();
    if (config_.loop.enabled)
        descriptor_.clear();
    frames_.invalidate();
    target_cache_.clear();
    next_epoch_seed_.reset();
    pending_epoch_anchor_ = std::monostate{};
    clearManualRelocalizationRequestLocked();
    last_lidar_observation_runtime_index_ = -1;
    last_tracking_reference_index_ = -1;
    last_recovery_timestamp_ = -std::numeric_limits<double>::infinity();
    pending_recovery_.reset();
    last_global_observation_cycle_time_ = Timestamp();
    last_recovery_cycle_time_ = Timestamp();
    last_loop_checked_index_ = -1;
    last_accepted_loop_index_ = -1;
    empty_loop_.reset();
    accepted_loops_.clear();
    loop_edges_.clear();
    publishOutputViewLocked();
}

/// 按最小周期推进调度水位，时钟回退时重建节流基准。
bool Localization::tryStartCycle(const Timestamp& now, const double target_period_sec,
                                 Timestamp& last_cycle_time) {
    if (!last_cycle_time.isZero() && now < last_cycle_time) {
        // 时钟回退时重新建立节流基准。
        last_cycle_time = Timestamp();
    }
    if (!last_cycle_time.isZero() && (now - last_cycle_time) < std::max(target_period_sec, 0.05)) {
        return false;
    }
    last_cycle_time = now;
    return true;
}

/// 初始化第零代输入、恢复种子及定位状态。
void Localization::initialize() {
    withOutputCommit([&] {
        next_epoch_seed_.reset();
        if (config_.startup_hint.T_map_sensor)
            next_epoch_seed_ = EpochSeed{*config_.startup_hint.T_map_sensor, false, {}};
        pending_epoch_anchor_ = std::monostate{};
        reset_coordinator_.clearRuntimeState();
        state_before_manual_reset_.reset();
        estimator_failed_ = false;
        frontend_status_ = FrontendStatus::Active;
        stationary_reset_ = false;
        odometry_connected_ = !config_.prior_map_enabled;
        active_epoch_ = 0;
        poses_.initialize();
        next_manual_relocalization_request_id_ = 0;
        clearManualRelocalizationRequestLocked();
        frames_.clear();
        last_lidar_observation_runtime_index_ = -1;
        last_global_observation_cycle_time_ = Timestamp();
        last_recovery_cycle_time_ = Timestamp();
        last_tracking_reference_index_ = -1;
        empty_loop_.reset();
        const bool pure_odometry = !config_.prior_map_enabled && !config_.loop.enabled &&
                                   !config_.startup_hint.T_map_sensor.has_value();
        if (pure_odometry) {
            poses_.setOrigin(config_.pure_odometry.origin_xy);
        }
        if (config_.prior_map_enabled) {
            if (config_.startup_hint.force_until_matched) {
                state_.initialize("waiting for forced startup-hint scan-to-map verification");
            } else {
                state_.initialize("waiting for startup-hint or descriptor scan-to-map");
            }
        } else {
            state_.initialize("waiting for first front-end frame");
        }
    });
}

/// 建立 SUSPECT 恢复源边界。
void Localization::beginSuspectRecoveryLocked() {
    frames_.beginRecovery();
    last_recovery_timestamp_ = -std::numeric_limits<double>::infinity();
    pending_recovery_.reset();
    last_recovery_cycle_time_ = Timestamp();
}

/// 为无先验地图的首个可信同步帧选择全局规范原因，并提交定位状态。
bool Localization::startTrajectoryLocked(const FrontendFrame& frame, bool applied_trusted_seed) {
    const char* startup_reason = "front-end trajectory started with map=odom";
    if (applied_trusted_seed) {
        startup_reason = "front-end epoch resumed from trusted map anchor";
    } else if (config_.startup_hint.T_map_sensor.has_value() && frame.epoch == 0) {
        startup_reason = "front-end trajectory started from startup hint";
    } else if (!config_.pure_odometry.origin_xy.isZero(1.0e-12)) {
        startup_reason =
            "front-end trajectory started from "
            "configured odometry gauge";
    }
    return state_.onTrustedCorrection(startup_reason, TrustedCorrectionSource::Relocalization)
        .changed;
}

/// 验收同步前端帧并在可信状态下提交轨迹与图更新。
bool Localization::ingestFrame(const FrontendFrame& frame,
                               const Eigen::Vector3d& linear_velocity_sensor,
                               const Eigen::Vector3d& angular_velocity_sensor) {
    bool state_changed = false;
    bool trajectory_changed = false;
    if (!withOutputCommit([&] {
            if (frontend_status_ != FrontendStatus::Active ||
                !frames_.accepts(frame, active_epoch_)) {
                return false;
            }
            bool applied_trusted_seed = false;
            if (!frames_.hasLatest() && next_epoch_seed_) {
                if (const auto* transform =
                        std::get_if<EpochTransform>(&next_epoch_seed_->origin)) {
                    if (frame.timestamp <= transform->boundary_stamp.seconds())
                        return false;
                    poses_.setTransform(transform->map_from_odom);
                } else {
                    poses_.setTransform(std::get<Eigen::Matrix4d>(next_epoch_seed_->origin) *
                                        frame.odom_pose.inverse());
                }
                applied_trusted_seed = next_epoch_seed_->trusted;
                pending_epoch_anchor_ = std::move(next_epoch_seed_->anchor);
                bindResetAnchorLocked(frame);
                next_epoch_seed_.reset();
                poses_.advanceRevision();
            }
            frames_.append(frame);

            if (!config_.prior_map_enabled &&
                state_.status() == LocalizationStatus::Initializing) {
                state_changed = startTrajectoryLocked(frame, applied_trusted_seed);
            }

            if (config_.prior_map_enabled &&
                (linear_velocity_sensor.norm() > config_.tracking.suspicious_linear_speed ||
                 angular_velocity_sensor.norm() > config_.tracking.suspicious_angular_speed)) {
                const LocalizationTransition transition = state_.onVelocitySpike(
                    linear_velocity_sensor.norm(), angular_velocity_sensor.norm());
                state_changed = state_changed || transition.changed;
                if (transition.changed && transition.current == LocalizationStatus::Suspect) {
                    beginSuspectRecoveryLocked();
                }
            }

            const LocalizationStatus current = state_.status();
            const bool can_record = LocalizationStateMachine::poseTrusted(current);
            if (can_record && trajectory_.shouldCreateKeyframe(frame, config_.keyframes)) {
                const std::uint64_t generation_before = state_.generation();
                addTrajectoryKeyframeLocked(frame);
                state_changed = state_changed || state_.generation() != generation_before;
                trajectory_changed = true;
            }
            return true;
        }))
        return false;

    if (state_changed) {
        publishState();
    }
    if (trajectory_changed) {
        visualization_sink_.publishVisualization(visualization());
    }
    return true;
}
/// 顺序完成维护任务的捕获、准备、身份认领、锁外计算和提交。
template <class Work, class Matcher>
void Localization::execute(std::optional<Work> (Localization::*capture)(), Matcher& matcher) {
    const auto start = std::chrono::steady_clock::now();
    auto work = (this->*capture)();
    if (!work)
        return;
    if constexpr (!std::is_same_v<Work, LoopWork>) {
        matcher.prepare(*work);
        if (!claim(*work))
            return;
    }
    auto result = matcher.compute(*work);
    if constexpr (!std::is_same_v<Work, LoopWork>) {
        publishRegistrationDebug(result.debug_aligned_source, result.debug_target);
    }
    if constexpr (std::is_same_v<Work, RecoveryWork>) {
        commit(*work, result);
    } else {
        const auto elapsed =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start)
                .count();
        const auto committed = commit(*work, result);
        if constexpr (std::is_same_v<Work, GlobalObservationWork>) {
            reportTracking(*work, result, committed, config_, elapsed);
        } else
            reportLoop(*work, result, committed);
    }
}

/// 以单次准入执行恢复、跟踪或回环维护任务。
void Localization::runMaintenanceCycle() {
    SingleFlightGate guard(maintenance_running_);
    if (!guard)
        return;
    expireFrontendResetIfTimedOut();
    bool failed = false;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (frontend_status_ != FrontendStatus::Active)
            return;
        failed = estimator_failed_;
    }
    if (failed && !config_.prior_map_enabled) {
        triggerFrontendReset();
        return;
    }
    if (LocalizationStateMachine::recoveryPath(status())) {
        if (config_.prior_map_enabled)
            execute(&Localization::captureRecovery, recovery_);
        return;
    }
    if (config_.prior_map_enabled)
        execute(&Localization::captureTracking, tracking_);
    if (config_.loop.enabled)
        execute(&Localization::captureLoop, loop_cycle_);
}
}  // namespace backend
