/**
 * @file localization_recovery.cpp
 * @brief 校验恢复任务身份并协调前端重置及可信锚提交。
 */

#include <cmath>

#include "backend/covariance.hpp"
#include "backend/localization.hpp"
#include "backend/logger.hpp"
#include "backend/math.hpp"

namespace backend {
/// 校验恢复任务的代次、状态及请求身份；不要求捕获帧仍是最新帧。
bool Localization::recoveryIsCurrentLocked(const RecoveryIdentity& identity) const {
    return active_epoch_ == identity.epoch && state_.status() == identity.status &&
           LocalizationStateMachine::recoveryPath(state_.status()) &&
           state_.generation() == identity.state_generation && !reset_coordinator_.pending() &&
           reset_coordinator_.lastSentTransactionId() == identity.reset_transaction_id &&
           (manual_request_ ? manual_request_->id : 0) == identity.manual_request_id;
}

/// 仅在认领恢复任务时要求点云和时间身份仍对应最新同步帧。
bool Localization::sourceIsLatestLocked(const RecoveryWork& work) const {
    return frames_.hasLatest() && frames_.latest().epoch == work.frame.epoch &&
           std::abs(frames_.latest().timestamp - work.frame.timestamp) < 1e-9 &&
           frames_.latest().cloud_local == work.frame.cloud_local;
}

/// 确认同步帧严格晚于手动请求捕获的输入边界。
bool Localization::manualRequestAllowsFrameLocked(const FrontendFrame& frame) const {
    if (!manual_request_) {
        return true;
    }
    if (frame.epoch != manual_request_->epoch) {
        return frame.epoch > manual_request_->epoch;
    }
    return !std::isfinite(manual_request_->after_timestamp) ||
           frame.timestamp > manual_request_->after_timestamp;
}

/// 在第零代初始化期间构造待几何验证的启动提示。
std::optional<RelocalizationPositionPrior> Localization::makeStartupHintPriorLocked(
    const FrontendFrame& frame) const {
    if (!config_.prior_map_enabled || state_.status() != LocalizationStatus::Initializing ||
        frame.epoch != 0 || !config_.startup_hint.T_map_sensor.has_value()) {
        return std::nullopt;
    }

    const Eigen::Matrix4d& T_map_sensor = *config_.startup_hint.T_map_sensor;
    const Eigen::Matrix3d R_map_body =
        T_map_sensor.block<3, 3>(0, 0) * config_.T_sensor_body.block<3, 3>(0, 0);
    const double body_yaw = yawFromRotation(R_map_body);

    RelocalizationPositionPrior prior;
    prior.source = "startup_hint";
    prior.map_point = T_map_sensor.block<3, 1>(0, 3);
    prior.runtime_sensor_point = Eigen::Vector3d::Zero();
    prior.R_yawless_sensor = yawGuess(-body_yaw, Eigen::Vector3d::Zero()).block<3, 3>(0, 0) *
                             T_map_sensor.block<3, 3>(0, 0);
    prior.yaw_seed_rad = body_yaw;
    prior.allow_yaw_sweep_fallback = false;
    prior.search_radius = config_.recovery.suspect_search_radius;
    prior.max_candidates = config_.relocalization.relocalization_max_candidates;
    return prior;
}

/// 冻结当前恢复任务的身份、源历史及位置先验。
std::optional<RecoveryWork> Localization::captureRecovery() {
    std::lock_guard<std::mutex> lock(mutex_);
    RecoveryWork work;

    const LocalizationStatus current = state_.status();
    if (!LocalizationStateMachine::recoveryPath(current) || reset_coordinator_.pending() ||
        !frames_.hasLatest()) {
        return std::nullopt;
    }
    if (current == LocalizationStatus::Lost && !config_.recovery.automatic_when_lost &&
        !manual_request_) {
        return std::nullopt;
    }
    if (manual_request_.has_value() && !manualRequestAllowsFrameLocked(frames_.latest())) {
        return std::nullopt;
    }
    if (std::abs(frames_.latest().timestamp - last_recovery_timestamp_) < 1.0e-9) {
        return std::nullopt;
    }

    work.frame = frames_.latest();
    work.runtime.index =
        !trajectory_.empty() && trajectory_.latest().epoch == frames_.latest().epoch &&
                std::abs(trajectory_.latest().timestamp - frames_.latest().timestamp) <= 1.0e-3
            ? trajectory_.latest().index
            : trajectory_.nextIndex();
    work.runtime.epoch = frames_.latest().epoch;
    work.runtime.timestamp = frames_.latest().timestamp;
    work.runtime.odom_pose = frames_.latest().odom_pose;
    work.identity.status = current;
    work.identity.state_generation = state_.generation();
    work.identity.reset_transaction_id = reset_coordinator_.lastSentTransactionId();
    work.identity.manual_request_id = manual_request_ ? manual_request_->id : 0;
    work.identity.epoch = active_epoch_;
    work.predicted_map_pose = poses_.transform() * frames_.latest().odom_pose;
    if (current == LocalizationStatus::Suspect && poses_.trusted().hasPose()) {
        // SUSPECT 只在最后可信位置附近召回；LOST 保持宽域或全局搜索。
        work.predicted_map_pose = poses_.trusted().mapSensorPose();
    }
    work.startup_hint_prior = makeStartupHintPriorLocked(frames_.latest());
    work.force_startup_hint =
        work.startup_hint_prior.has_value() && config_.startup_hint.force_until_matched;
    work.source_history.assign(frames_.history().begin(), frames_.history().end());
    return work;
}

/// 安装恢复位姿锚，并把匹配期间的新里程计运动传播到当前可信输出。
PoseCorrectionDelta Localization::applyRecoveryLocked(const RecoveryWork& work,
                                                      const RecoveryResult& result,
                                                      LocalizationTransition& transition) {
    PoseCorrectionDelta applied_correction;
    Eigen::Matrix4d output_pose_before = poses_.transform() * frames_.latest().odom_pose;
    if (poses_.trusted().hasPose()) {
        output_pose_before = poses_.trusted().mapSensorPose();
    }
    // 恢复观测属于捕获帧，同状态代内的新帧继续使用校正后的全局规范。
    poses_.setTransform(result.pose * work.frame.odom_pose.inverse());
    anchorRecoveredFrameLocked(work.frame, result.pose, result.covariance, work.source_history);
    odom_gate_anchor_.reset();
    odom_gate_trusted_epoch_ = work.frame.epoch;
    odom_gate_trusted_since_ = work.frame.timestamp;
    last_tracking_reference_index_ = -1;
    last_lidar_observation_runtime_index_ = trajectory_.latest().index;
    last_global_observation_cycle_time_ = Timestamp();
    clearManualRelocalizationRequestLocked();
    transition = state_.onTrustedCorrection("recovered via " + result.source,
                                            TrustedCorrectionSource::Relocalization);
    // 恢复可能锁到对称或偏移位姿：试用期内一次严重冲突即重新进入 SUSPECT。
    state_.beginProbation(config_.recovery.probation_observations);
    // 将匹配期间到达的里程计运动传播到最新同步帧，并提交可信输出。
    commitCurrentTrustedPoseLocked();
    const Eigen::Matrix4d output_pose_after = poses_.transform() * frames_.latest().odom_pose;
    const Eigen::Matrix4d correction = output_pose_before.inverse() * output_pose_after;
    if (correction.allFinite()) {
        applied_correction.translation = correction.block<3, 1>(0, 3).norm();
        applied_correction.rotation_rad = rotationAngleFromMatrix(correction);
    }
    return applied_correction;
}

/// 累计恢复候选；连续给出一致 map←odom 变换达到配置次数后返回 true。
/// 相似场景下单帧匹配可能给出错误位姿，要求多帧一致可在提交前将其滤除。
bool Localization::recoveryConfirmedLocked(const RecoveryWork& work,
                                           const RecoveryResult& result) {
    const auto& rc = config_.recovery;
    const int required = rc.confirmations;
    const bool motion_gate = rc.confirm_min_translation > 0.0 || rc.confirm_min_rotation_rad > 0.0;
    if (required <= 1 && !motion_gate)
        return true;
    const Eigen::Matrix4d T_map_odom = result.pose * work.frame.odom_pose.inverse();
    /// 两个 map←odom 变换在确认门限内一致。
    const auto consistent = [&](const Eigen::Matrix4d& reference, double& translation,
                                double& rotation) {
        const Eigen::Matrix4d delta = reference.inverse() * T_map_odom;
        translation = delta.block<3, 1>(0, 3).norm();
        rotation = rotationAngleFromMatrix(delta);
        return delta.allFinite() && translation <= rc.confirm_translation &&
               rotation <= rc.confirm_rotation_rad;
    };
    if (pending_recovery_ && pending_recovery_->epoch == work.identity.epoch) {
        double translation = 0.0, rotation = 0.0, anchor_t = 0.0, anchor_r = 0.0;
        // 同时与上一候选和首个候选比较：防止一致性沿多次候选逐步漂移。
        if (consistent(pending_recovery_->T_map_odom, translation, rotation) &&
            consistent(pending_recovery_->first_T_map_odom, anchor_t, anchor_r)) {
            ++pending_recovery_->confirmations;
            pending_recovery_->T_map_odom = T_map_odom;
            const Eigen::Matrix4d motion =
                pending_recovery_->first_odom_pose.inverse() * work.frame.odom_pose;
            const double moved_t = motion.block<3, 1>(0, 3).norm();
            const double moved_r = rotationAngleFromMatrix(motion);
            const bool moved =
                !motion_gate ||
                (rc.confirm_min_translation > 0.0 && moved_t >= rc.confirm_min_translation) ||
                (rc.confirm_min_rotation_rad > 0.0 && moved_r >= rc.confirm_min_rotation_rad);
            const bool stationary_ok =
                rc.confirm_stationary_max > 0 &&
                pending_recovery_->confirmations >= rc.confirm_stationary_max;
            if (pending_recovery_->confirmations >= required && (moved || stationary_ok)) {
                BACKEND_INFO_STREAM("recovery candidate accepted after "
                                    << pending_recovery_->confirmations
                                    << " consistent candidates, motion=" << moved_t << "m/"
                                    << moved_r * 180.0 / M_PI << "deg"
                                    << (moved ? "" : " (stationary limit)"));
                pending_recovery_.reset();
                return true;
            }
            BACKEND_INFO_STREAM("recovery candidate confirmed "
                                << pending_recovery_->confirmations << "/" << required
                                << ", motion=" << moved_t << "m/" << moved_r * 180.0 / M_PI
                                << "deg via " << result.source);
            return false;
        }
        BACKEND_INFO_STREAM("recovery candidate inconsistent with previous: delta_t="
                            << translation << "m, delta_r_deg=" << rotation * 180.0 / M_PI
                            << ", vs first: delta_t=" << anchor_t << "m, delta_r_deg="
                            << anchor_r * 180.0 / M_PI << "; restarting confirmation");
    }
    pending_recovery_ =
        PendingRecovery{T_map_odom, 1, work.identity.epoch, T_map_odom, work.frame.odom_pose};
    BACKEND_INFO_STREAM("recovery candidate awaiting confirmation 1/" << required << " via "
                                                                    << result.source);
    return false;
}

/// 里程计连续性门：同一前端代次内，里程计是连续的，进入 SUSPECT 时的 map←odom 乘当前里程计即为预测位姿。
/// 里程计只走了很短距离而恢复解离预测很远（中心对称场地的对称位姿、远处相似场景）时拒绝，
/// 不计为恢复失败。连续拒绝达到上限后门失效：里程计本身可能已失真（碰撞、被搬动）。
bool Localization::recoveryConsistentWithOdometryLocked(const RecoveryWork& work,
                                                       const RecoveryResult& result) {
    const auto& rc = config_.recovery;
    if (rc.odom_gate_translation <= 0.0 && rc.odom_gate_rotation_rad <= 0.0)
        return true;
    if (!odom_gate_anchor_ || odom_gate_anchor_->epoch != work.frame.epoch ||
        (work.identity.status != LocalizationStatus::Suspect &&
         work.identity.status != LocalizationStatus::Lost))
        return true;
    OdomGateAnchor& anchor = *odom_gate_anchor_;
    if (rc.odom_gate_max_rejections > 0 && anchor.rejections >= rc.odom_gate_max_rejections)
        return true;

    const Eigen::Matrix4d travel = anchor.odom_pose.inverse() * work.frame.odom_pose;
    const double travel_t = travel.block<3, 1>(0, 3).norm();
    const Eigen::Matrix4d predicted = anchor.T_map_odom * work.frame.odom_pose;
    const Eigen::Matrix4d delta = predicted.inverse() * result.pose;
    if (!delta.allFinite())
        return true;
    const double offset_t = delta.block<2, 1>(0, 3).norm();
    const double offset_r = rotationAngleFromMatrix(delta);
    const double limit_t = rc.odom_gate_translation + rc.odom_gate_translation_ratio * travel_t;
    const bool ok_t = rc.odom_gate_translation <= 0.0 || offset_t <= limit_t;
    const bool ok_r = rc.odom_gate_rotation_rad <= 0.0 || offset_r <= rc.odom_gate_rotation_rad;
    if (ok_t && ok_r)
        return true;
    ++anchor.rejections;
    BACKEND_WARN_STREAM("recovery candidate rejected: inconsistent with odometry since SUSPECT "
                        << "(offset=" << offset_t << "m/" << offset_r * 180.0 / M_PI
                        << "deg, limit=" << limit_t << "m/" << rc.odom_gate_rotation_rad * 180.0 / M_PI
                        << "deg, odom travel=" << travel_t << "m, via " << result.source
                        << ", rejection " << anchor.rejections << "/" << rc.odom_gate_max_rejections
                        << (rc.odom_gate_max_rejections > 0 &&
                                    anchor.rejections >= rc.odom_gate_max_rejections
                                ? "; odometry gate released"
                                : "")
                        << ")");
    return false;
}

/// 校验恢复任务身份并提交完整位姿锚或累计恢复失败。
void Localization::commit(const RecoveryWork& work, const RecoveryResult& result) {
    if (result.outcome == RecoveryOutcome::Deferred)
        return;
    const bool matched = result.outcome == RecoveryOutcome::Matched;
    PoseCorrectionDelta applied_correction;
    LocalizationTransition transition;
    if (!withOutputCommit([&] {
            if (!recoveryIsCurrentLocked(work.identity)) {
                BACKEND_INFO_STREAM("discarding stale recovery result: runtime="
                                    << work.runtime.index << ", epoch=" << work.frame.epoch
                                    << ", generation=" << work.identity.state_generation
                                    << ", reset_transaction=" << work.identity.reset_transaction_id
                                    << "->" << reset_coordinator_.lastSentTransactionId()
                                    << ", manual_request=" << work.identity.manual_request_id);
                return false;
            }
            if (matched) {
                // 里程计很短却跳得很远的解（对称位姿）不参与确认，也不计为失败。
                if (!recoveryConsistentWithOdometryLocked(work, result)) {
                    pending_recovery_.reset();
                    return false;
                }
                // 未获连续确认的候选只记录，不改变状态与输出。
                if (!recoveryConfirmedLocked(work, result))
                    return false;
                applied_correction = applyRecoveryLocked(work, result, transition);
                estimator_failed_ = false;
            } else {
                pending_recovery_.reset();
                if (work.identity.manual_request_id != 0)
                    clearManualRelocalizationRequestLocked();
                transition = state_.onRecoveryRelocalizationFailure(
                    config_.recovery.suspect_failures_before_lost, result.source);
            }
            return true;
        }))
        return;
    publishState();
    if (matched) {
        visualization_sink_.publishVisualization(visualization());
        BACKEND_INFO_STREAM("localization state -> "
                            << LocalizationStateMachine::statusName(transition.current) << " ("
                            << transition.reason << ", source_frames=" << work.accumulated_frames
                            << "/" << config_.recovery.source.max_frames
                            << ", source_span=" << work.source_span_seconds
                            << "s, delta_t=" << applied_correction.translation << "m, delta_r_deg="
                            << applied_correction.rotation_rad * 180.0 / M_PI << ")");
    } else {
        BACKEND_WARN_STREAM("localization recovery pending: " << transition.reason);
        if (transition.current == LocalizationStatus::Lost)
            triggerFrontendReset();
    }
}
/// 复核恢复身份与最新源帧，并按节流规则认领任务。
bool Localization::claim(RecoveryWork& work) {
    const bool waiting = !work.startup_hint_prior &&
                         work.source_evidence.disposition == ObservationDisposition::Unavailable;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto current = state_.status();
        const bool same = recoveryIsCurrentLocked(work.identity) && sourceIsLatestLocked(work);
        if (!waiting) {
            if (!same)
                return false;
            if (current == LocalizationStatus::Lost && !config_.recovery.automatic_when_lost &&
                work.identity.manual_request_id == 0)
                return false;
            if (!work.force_startup_hint &&
                !tryStartCycle(clock_.nowSteadyTimestamp(), config_.recovery.period,
                               last_recovery_cycle_time_))
                return false;
        }
        // 等待新源帧不消费周期或手动请求；旧帧在这一边界后不会反复检查。
        if (same)
            last_recovery_timestamp_ = work.frame.timestamp;
    }
    if (waiting && work.identity.manual_request_id != 0) {
        BACKEND_INFO_STREAM("manual relocalization request "
                            << work.identity.manual_request_id << " waiting for recovery source ("
                            << work.accumulated_frames << "/" << config_.recovery.source.min_frames
                            << " frames, span=" << work.source_span_seconds << "s)");
    }
    return !waiting;
}
/// 在 LOST 状态记录等待下一有效帧的手动恢复请求。
bool Localization::requestManualRelocalization() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_.status() != LocalizationStatus::Lost) {
        return false;
    }
    std::uint64_t request_id = ++next_manual_relocalization_request_id_;
    if (request_id == 0) {
        request_id = ++next_manual_relocalization_request_id_;
    }
    manual_request_ = ManualRequest{request_id, active_epoch_,
                                    frames_.hasLatest() ? frames_.latest().timestamp
                                                        : std::numeric_limits<double>::quiet_NaN()};
    return true;
}

/// 清除已经完成或失效的手动恢复请求。
void Localization::clearManualRelocalizationRequestLocked() {
    manual_request_.reset();
}

}  // namespace backend
