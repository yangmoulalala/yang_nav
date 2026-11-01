/**
 * @file fusion_parameters.cpp
 * @brief 读取 UWB、回环、图估计、重置与启动提示参数。
 */

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "apps/parameters.hpp"
#include "backend/math.hpp"

namespace backend::ros {

/// 读取 UWB 坐标、时间、噪声、外参和启动对齐参数。
void ParameterReader::loadUwb() {
    getParam("uwb/enabled", parameters_.runtime.uwb.enabled);
    getParam("uwb/use_z", parameters_.runtime.uwb.use_z);
    getParam("uwb/frame", parameters_.input.uwb_frame);
    if (parameters_.input.uwb_frame.empty()) {
        throw std::invalid_argument("uwb.frame must not be empty");
    }
    loadUwbFrameCalibration();
    getParam("uwb/effective_rate_hz", parameters_.runtime.uwb.effective_rate_hz);
    getParam("uwb/time_offset_seconds", parameters_.runtime.uwb.time_offset_seconds);
    getParam("uwb/estimate_time_offset_online",
             parameters_.runtime.uwb.estimate_time_offset_online);
    getParam("uwb/time_offset_prior_std_seconds",
             parameters_.runtime.uwb.time_offset_prior_std_seconds);
    getParam("uwb/time_jitter_std_seconds", parameters_.runtime.uwb.time_jitter_std_seconds);
    getParam("uwb/max_age", parameters_.runtime.uwb.max_age);
    getParam("uwb/odom_sync_tolerance", parameters_.runtime.uwb.odom_sync_tolerance);
    getParam("uwb/recovery_search_radius", parameters_.runtime.uwb.recovery_search_radius);
    getParam("uwb/recovery_candidates", parameters_.runtime.uwb.recovery_candidates);
    loadUwbMeasurementModel();
    loadUwbBootstrap();
}

/// 读取在线坐标标定开关及固定 UWB 到地图坐标变换。
void ParameterReader::loadUwbFrameCalibration() {
    getParam("uwb/estimate_zero_pose_online", parameters_.runtime.uwb.estimate_zero_pose_online);
    Eigen::Vector3d uwb_zero_position = Eigen::Vector3d::Zero();
    double uwb_zero_yaw_deg = 0.0;
    getParam("uwb/zero_pose/x", uwb_zero_position.x());
    getParam("uwb/zero_pose/y", uwb_zero_position.y());
    getParam("uwb/zero_pose/z", uwb_zero_position.z());
    getParam("uwb/zero_pose/yaw_deg", uwb_zero_yaw_deg);
    if (!uwb_zero_position.allFinite() || !std::isfinite(uwb_zero_yaw_deg)) {
        throw std::invalid_argument("uwb.zero_pose calibration contains a non-finite value");
    }
    parameters_.runtime.uwb.T_map_uwb_fixed =
        yawGuess(degreesToRadians(uwb_zero_yaw_deg), uwb_zero_position);
}

/// 读取天线机体位置并构造位置测量协方差；换算到传感器系由 applyBodyExtrinsic() 完成。
void ParameterReader::loadUwbMeasurementModel() {
    getParam("uwb/antenna_position_body/x", parameters_.input.antenna_position_body.x());
    getParam("uwb/antenna_position_body/y", parameters_.input.antenna_position_body.y());
    getParam("uwb/antenna_position_body/z", parameters_.input.antenna_position_body.z());
    if (!parameters_.input.antenna_position_body.allFinite()) {
        throw std::invalid_argument("uwb.antenna_position_body contains a non-finite value");
    }
    double uwb_xy_std = 0.2;
    double uwb_z_std = 0.3;
    getParam("uwb/position_std_xy", uwb_xy_std);
    getParam("uwb/position_std_z", uwb_z_std);
    if (!std::isfinite(uwb_xy_std) || uwb_xy_std <= 0.0) {
        RCLCPP_WARN(node_.get_logger(),
                    "uwb.position_std_xy must be finite and positive; using "
                    "0.2 m");
        uwb_xy_std = 0.2;
    }
    if (!std::isfinite(uwb_z_std) || uwb_z_std <= 0.0) {
        RCLCPP_WARN(node_.get_logger(),
                    "uwb.position_std_z must be finite and positive; using "
                    "0.3 m");
        uwb_z_std = 0.3;
    }
    parameters_.runtime.uwb.position_covariance =
        Eigen::Vector3d(uwb_xy_std * uwb_xy_std, uwb_xy_std * uwb_xy_std, uwb_z_std * uwb_z_std)
            .asDiagonal();
}

/// 读取启动配对、鲁棒验收及稳定性参数，并转换偏航和重力角度单位。
void ParameterReader::loadUwbBootstrap() {
    getParam("uwb/bootstrap/max_pairs", parameters_.runtime.uwb.bootstrap.max_pairs);
    getParam("uwb/bootstrap/min_pairs", parameters_.runtime.uwb.bootstrap.min_pairs);
    getParam("uwb/bootstrap/min_baseline", parameters_.runtime.uwb.bootstrap.min_baseline);
    getParam("uwb/bootstrap/inlier_threshold", parameters_.runtime.uwb.bootstrap.inlier_threshold);
    getParam("uwb/bootstrap/max_rmse", parameters_.runtime.uwb.bootstrap.max_rmse);
    double bootstrap_max_yaw_std_deg = 10.0;
    getParam("uwb/bootstrap/max_yaw_std_deg", bootstrap_max_yaw_std_deg);
    parameters_.runtime.uwb.bootstrap.max_yaw_std_rad = degreesToRadians(bootstrap_max_yaw_std_deg);
    getParam("uwb/bootstrap/stable_solutions", parameters_.runtime.uwb.bootstrap.stable_solutions);
    getParam("uwb/bootstrap/stable_translation_delta",
             parameters_.runtime.uwb.bootstrap.stable_translation_delta);
    double bootstrap_stable_yaw_delta_deg = 3.0;
    getParam("uwb/bootstrap/stable_yaw_delta_deg", bootstrap_stable_yaw_delta_deg);
    parameters_.runtime.uwb.bootstrap.stable_yaw_delta_rad =
        degreesToRadians(bootstrap_stable_yaw_delta_deg);
    double bootstrap_gravity_std_deg = 1.0;
    getParam("uwb/bootstrap/gravity_alignment_std_deg", bootstrap_gravity_std_deg);
    parameters_.runtime.uwb.bootstrap.gravity_alignment_std_rad =
        degreesToRadians(bootstrap_gravity_std_deg);
}

/// 读取回环验证与因子图噪声、鲁棒核和窗口参数。
void ParameterReader::loadLoopAndGraph() {
    getParam("loop/enabled", parameters_.runtime.loop.enabled);
    getParam("loop/cooldown_keyframes", parameters_.runtime.loop.cooldown_keyframes);
    getParam("loop/candidate_max_distance", parameters_.runtime.loop.candidate_max_distance);
    getParam("loop/descriptor_ratio_threshold",
             parameters_.runtime.loop.descriptor_ratio_threshold);
    getParam("loop/source/min_frames", parameters_.runtime.loop.source.min_frames);
    getParam("loop/source/max_frames", parameters_.runtime.loop.source.max_frames);
    getParam("loop/source/max_time", parameters_.runtime.loop.source.max_time);
    getParam("loop/source/max_translation", parameters_.runtime.loop.source.max_translation);
    double loop_source_max_rotation_deg = 0.0;
    getParam("loop/source/max_rotation_deg", loop_source_max_rotation_deg);
    parameters_.runtime.loop.source.max_rotation_rad =
        degreesToRadians(loop_source_max_rotation_deg);
    int loop_target_keyframes = 16;
    getParam("loop/target_keyframes", loop_target_keyframes);
    getParam("loop/target_radius", parameters_.runtime.loop.target_radius);
    parameters_.runtime.loop.target_keyframes =
        static_cast<std::size_t>(std::max(1, loop_target_keyframes));
    getParam("loop/max_parallel_verifications",
             parameters_.runtime.loop.max_parallel_verifications);
    getParam("loop/history_size", parameters_.runtime.loop.history_size);

    LoopClosureConfig& loop_validation = parameters_.runtime.loop.verification;
    getParam("loop/validation/max_translation", loop_validation.translation_max);
    getParam("loop/validation/max_z", loop_validation.z_max);
    double loop_roll_pitch_deg = 15.0;
    double loop_yaw_deg = 45.0;
    getParam("loop/validation/max_roll_pitch_deg", loop_roll_pitch_deg);
    getParam("loop/validation/max_yaw_residual_deg", loop_yaw_deg);
    loop_validation.roll_pitch_max_rad = degreesToRadians(loop_roll_pitch_deg);
    loop_validation.yaw_residual_max_rad = degreesToRadians(loop_yaw_deg);
    getParam("loop/validation/overlap_distance", loop_validation.overlap_distance);
    getParam("loop/validation/min_overlap_ratio", loop_validation.min_overlap_ratio);
    getParam("loop/pcm/enabled", loop_validation.enable_pcm);
    getParam("loop/pcm/translation_threshold", loop_validation.pcm_translation_threshold);
    double pcm_rotation_deg = 20.0;
    getParam("loop/pcm/rotation_threshold_deg", pcm_rotation_deg);
    loop_validation.pcm_rotation_threshold_rad = degreesToRadians(pcm_rotation_deg);
    getParam("loop/pcm/min_consistent_loops", loop_validation.min_consistent_loops);
    getParam("loop/pcm/min_candidate_clique_size", loop_validation.min_candidate_clique_size);

    loadGraph();
}

/// 读取图先验、里程计过程噪声、鲁棒核及窗口长度，并规范化鲁棒核配置。
void ParameterReader::loadGraph() {
    PoseGraphConfig& graph = parameters_.runtime.graph;
    double prior_rotation_variance = 1.0e-4;
    double prior_translation_variance = 1.0e-2;
    getParam("graph/prior_rotation_variance", prior_rotation_variance);
    getParam("graph/prior_translation_variance", prior_translation_variance);
    getParam("graph/odom_noise/rotation_variance_per_second",
             graph.odom_noise.rotation_variance_per_second);
    getParam("graph/odom_noise/translation_variance_per_second",
             graph.odom_noise.translation_variance_per_second);
    getParam("graph/odom_noise/rotation_variance_per_radian",
             graph.odom_noise.rotation_variance_per_radian);
    getParam("graph/odom_noise/translation_variance_per_meter",
             graph.odom_noise.translation_variance_per_meter);
    graph.prior_rotation_variances.setConstant(prior_rotation_variance);
    graph.prior_translation_variances.setConstant(prior_translation_variance);
    getParam("graph/robust/enabled", graph.use_robust_kernel);
    getParam("graph/robust/kernel", graph.robust_kernel);
    getParam("graph/robust/size", graph.robust_kernel_size);
    getParam("graph/fixed_lag_seconds", graph.fixed_lag_seconds);
    if (graph.robust_kernel != "huber" && graph.robust_kernel != "cauchy") {
        RCLCPP_WARN_STREAM(node_.get_logger(), "unknown graph.robust.kernel '"
                                                   << graph.robust_kernel << "'; using huber");
        graph.robust_kernel = "huber";
    }
    graph.robust_kernel_size = std::max(1.0e-6, graph.robust_kernel_size);
}

/// 读取幂等 reset、纯里程计 gauge 及 epoch 0 启动提示。
void ParameterReader::loadResetAndStartup() {
    getParam("reset/service", parameters_.services.frontend_reset);
    getParam("reset/min_gap", parameters_.runtime.reset.min_gap_sec);
    getParam("reset/response_timeout", parameters_.runtime.reset.response_timeout_sec);
    getParam("reset/stationary_translation_std",
             parameters_.runtime.reset.stationary_translation_std);
    double reset_stationary_rotation_std_deg = 1.0;
    getParam("reset/stationary_rotation_std_deg", reset_stationary_rotation_std_deg);
    parameters_.runtime.reset.stationary_rotation_std_rad =
        degreesToRadians(reset_stationary_rotation_std_deg);

    getParam("pure_odometry/origin_x", parameters_.runtime.pure_odometry.origin_xy.x());
    getParam("pure_odometry/origin_y", parameters_.runtime.pure_odometry.origin_xy.y());

    bool startup_hint_enabled = false;
    Eigen::Vector3d startup_position = Eigen::Vector3d::Zero();
    getParam("startup_hint/enabled", startup_hint_enabled);
    getParam("startup_hint/force_until_matched",
             parameters_.runtime.startup_hint.force_until_matched);
    getParam("startup_hint/x", startup_position.x());
    getParam("startup_hint/y", startup_position.y());
    getParam("startup_hint/z", startup_position.z());
    double startup_yaw_deg = 0.0;
    getParam("startup_hint/yaw_deg", startup_yaw_deg);
    // 启动提示描述 body 位姿；换算为 T_map_sensor 依赖 TF 外参，由 applyBodyExtrinsic() 完成。
    if (startup_hint_enabled) {
        if (!startup_position.allFinite() || !std::isfinite(startup_yaw_deg)) {
            throw std::invalid_argument("startup_hint contains a non-finite value");
        }
        startup_T_map_body_ = yawGuess(degreesToRadians(startup_yaw_deg), startup_position);
    } else {
        startup_T_map_body_.reset();
    }
}

}  // namespace backend::ros
