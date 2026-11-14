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
    prior.search_radius = config_.recovery.initial_search_radius;
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
        // SUSPECT 以最后可信 map←odom 前推的位姿为预测（UWB 先验的高度取自预测位姿）。
        work.predicted_map_pose = poses_.trusted()
                                      .project(current, poses_.transform(),
                                               frames_.latest().odom_pose, frames_.latest().epoch)
                                      .T_map_sensor;
    }
    work.startup_hint_prior = makeStartupHintPriorLocked(frames_.latest());
    work.force_startup_hint =
        work.startup_hint_prior.has_value() && config_.startup_hint.force_until_matched;
    // 局中恢复只用 UWB 位置先验；开机初始化不用 UWB（启动提示 / 起点附近描述子）。
    work.uwb_required =
        current == LocalizationStatus::Suspect || current == LocalizationStatus::Lost;
    if (work.uwb_required)
        work.uwb_prior = makeUwbRecoveryPriorLocked(frames_.latest());
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
        // 与恢复前实际发布的输出比较：SUSPECT 为前推位姿，LOST 为冻结位姿。
        output_pose_before = poses_.trusted()
                                 .project(state_.status(), poses_.transform(),
                                          frames_.latest().odom_pose, frames_.latest().epoch)
                                 .T_map_sensor;
    }
    // 恢复观测属于捕获帧，同状态代内的新帧继续使用校正后的全局规范。
    poses_.setTransform(result.pose * work.frame.odom_pose.inverse());
    anchorRecoveredFrameLocked(work.frame, result.pose, result.covariance, work.source_history);
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

/// Build a fresh UWB position prior with the antenna lever arm at measurement time.
/// Same-epoch SUSPECT tries continuous heading first; LOST retains the full yaw sweep.
std::optional<RelocalizationPositionPrior> Localization::makeUwbRecoveryPriorLocked(
    const FrontendFrame& frame) const {
    if (!config_.uwb.enabled || !latest_uwb_ || !T_map_uwb_ || frames_.odometry().empty())
        return std::nullopt;
    const double motion_timestamp = latest_uwb_->timestamp + config_.uwb.time_offset_seconds;
    const double age = frame.timestamp - motion_timestamp;
    if (age < -config_.uwb.odom_sync_tolerance || age > config_.uwb.max_age)
        return std::nullopt;
    const std::optional<Eigen::Matrix4d> timed_odom_pose =
        frames_.odometry().poseAt(motion_timestamp, config_.uwb.odom_sync_tolerance);
    if (!timed_odom_pose)
        return std::nullopt;

    const Eigen::Matrix4d T_sensor_body = latest_uwb_->T_sensor_body.value_or(config_.T_sensor_body);
    const Eigen::Vector3d antenna_sensor =
        T_sensor_body.block<3, 3>(0, 0) * config_.uwb.antenna_position_body +
        T_sensor_body.block<3, 1>(0, 3);
    const Eigen::Matrix3d R_odom_sensor = frame.odom_pose.block<3, 3>(0, 0);
    const double body_yaw = yawFromRotation(R_odom_sensor * T_sensor_body.block<3, 3>(0, 0));

    RelocalizationPositionPrior prior;
    prior.source = "uwb";
    prior.use_z = false;
    prior.map_point = T_map_uwb_->block<3, 3>(0, 0) * latest_uwb_->position_uwb +
                      T_map_uwb_->block<3, 1>(0, 3);
    prior.R_yawless_sensor =
        yawGuess(-body_yaw, Eigen::Vector3d::Zero()).block<3, 3>(0, 0) * R_odom_sensor;
    const Eigen::Vector3d antenna_odom =
        timed_odom_pose->block<3, 3>(0, 0) * antenna_sensor + timed_odom_pose->block<3, 1>(0, 3);
    prior.runtime_sensor_point =
        R_odom_sensor.transpose() * (antenna_odom - frame.odom_pose.block<3, 1>(0, 3));
    prior.yaw_seed_rad.reset();
    // SUSPECT still has a continuous same-epoch orientation estimate. UWB only
    // measures position: do not discard that heading and start with an unconstrained
    // full-circle search through similar tunnel geometry. The existing search
    // falls back to the full sweep if this seed cannot establish a valid anchor.
    if (state_.status() == LocalizationStatus::Suspect && poses_.trusted().hasPose() &&
        poses_.trusted().epoch() == frame.epoch) {
        const Eigen::Matrix4d predicted = poses_.trusted()
                                              .project(state_.status(), poses_.transform(),
                                                       frame.odom_pose, frame.epoch)
                                              .T_map_sensor;
        const double predicted_yaw = yawFromRotation(
            predicted.block<3, 3>(0, 0) * T_sensor_body.block<3, 3>(0, 0));
        prior.R_yawless_sensor =
            yawGuess(-predicted_yaw, Eigen::Vector3d::Zero()).block<3, 3>(0, 0) *
            predicted.block<3, 3>(0, 0);
        prior.yaw_seed_rad = predicted_yaw;
    }
    prior.search_radius = config_.uwb.search_radius;
    prior.max_candidates = config_.uwb.candidates;
    if (!prior.map_point.allFinite() || !prior.runtime_sensor_point.allFinite())
        return std::nullopt;
    return prior;
}

/// UWB 先验决定召回范围；这里再用同一天线点检查恢复解与 UWB 的水平偏差。
bool Localization::recoveryConsistentWithUwbLocked(const RecoveryWork& work,
                                                   const RecoveryResult& result) const {
    if (!work.uwb_prior || config_.uwb.max_offset <= 0.0)
        return true;
    const RelocalizationPositionPrior& prior = *work.uwb_prior;
    // result.pose 为捕获帧的 T_map_sensor；runtime_sensor_point 为同一帧传感器系中的天线点。
    const Eigen::Vector3d antenna_map =
        result.pose.block<3, 3>(0, 0) * prior.runtime_sensor_point + result.pose.block<3, 1>(0, 3);
    const double offset = (antenna_map - prior.map_point).head<2>().norm();
    if (std::isfinite(offset) && offset <= config_.uwb.max_offset)
        return true;
    BACKEND_WARN_STREAM("recovery candidate rejected: inconsistent with UWB (offset="
                        << offset << "m > " << config_.uwb.max_offset << "m, via " << result.source
                        << ")");
    return false;
}

/// 累计恢复候选；连续 recovery.confirmations 个候选的 map←odom 一致后返回 true。
/// 每个新候选须与上一个和第一个候选都一致，防止一致性沿候选链逐步漂移。
bool Localization::recoveryConfirmedLocked(const RecoveryWork& work,
                                           const RecoveryResult& result) {
    const auto& rc = config_.recovery;
    const int required = rc.confirmations;
    if (required <= 1)
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
        if (consistent(pending_recovery_->T_map_odom, translation, rotation) &&
            consistent(pending_recovery_->first_T_map_odom, anchor_t, anchor_r)) {
            ++pending_recovery_->confirmations;
            pending_recovery_->T_map_odom = T_map_odom;
            if (pending_recovery_->confirmations >= required) {
                BACKEND_INFO_STREAM("recovery candidate accepted after "
                                    << pending_recovery_->confirmations
                                    << " consistent candidates via " << result.source);
                pending_recovery_.reset();
                return true;
            }
            BACKEND_INFO_STREAM("recovery candidate confirmed "
                                << pending_recovery_->confirmations << "/" << required << " via "
                                << result.source);
            return false;
        }
        BACKEND_INFO_STREAM("recovery candidate inconsistent with previous: delta_t="
                            << translation << "m, delta_r_deg=" << rotation * 180.0 / M_PI
                            << ", vs first: delta_t=" << anchor_t << "m, delta_r_deg="
                            << anchor_r * 180.0 / M_PI << "; restarting confirmation");
    }
    pending_recovery_ = PendingRecovery{T_map_odom, 1, work.identity.epoch, T_map_odom};
    BACKEND_INFO_STREAM("recovery candidate awaiting confirmation 1/" << required << " via "
                                                                    << result.source);
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
                // 与 UWB 读数不一致的解（附近相似关键帧）不参与确认，也不计为失败。
                if (!recoveryConsistentWithUwbLocked(work, result)) {
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
