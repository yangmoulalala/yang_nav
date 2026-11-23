/**
 * @file config.cpp
 * @brief 校验并规范化激光惯性里程计的算法参数。
 */
#include "zlio/config.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace zlio {
namespace {
/// 校验数值为有限正数。
void positive(double value, const char* name) {
    if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
}

/// 按配置路径校验惯性测量和偏置噪声。
void validateImuNoise(const ImuNoise& noise, const std::string& path) {
    positive(noise.gyro_sample_variance, (path + ".gyro_variance").c_str());
    positive(noise.accel_sample_variance, (path + ".accel_variance").c_str());
    positive(noise.gyro_bias_psd, (path + ".gyro_bias_psd").c_str());
    positive(noise.accel_bias_psd, (path + ".accel_bias_psd").c_str());
}

/// 校验采样尺度、目标数量和反馈增益。
void validateSampling(const SamplingConfig& s) {
    positive(s.minimum_size, "sampling.minimum_size");
    positive(s.maximum_size, "sampling.maximum_size");
    positive(s.initial_size, "sampling.initial_size");
    positive(s.minimum_points, "sampling.minimum_points");
    positive(s.maximum_points, "sampling.maximum_points");
    positive(s.scale_threshold, "sampling.scale_threshold");
    positive(s.proportional_scale, "sampling.proportional_scale");
    positive(s.derivative_scale, "sampling.derivative_scale");
    positive(s.proportional_minimum, "sampling.proportional_minimum");
    positive(s.proportional_maximum, "sampling.proportional_maximum");
    positive(s.derivative_minimum, "sampling.derivative_minimum");
    positive(s.derivative_maximum, "sampling.derivative_maximum");
    if (s.history_size == 0 || s.minimum_size > s.initial_size || s.initial_size > s.maximum_size ||
        s.minimum_points > s.maximum_points || !std::isfinite(s.exponent) || s.exponent <= 1 ||
        s.proportional_minimum > s.proportional_maximum ||
        s.derivative_minimum > s.derivative_maximum)
        throw std::invalid_argument("invalid sampling parameter range");
}

/// 校验启动窗口和粗配准设置。
void validateInitialization(const InitializationConfig& config) {
    if (config.window_frames < 2 || config.window_frames > std::numeric_limits<int>::max() / 15 ||
        config.seed_imu_samples == 0 || config.maximum_refinements < 4 ||
        config.minimum_sampled_points == 0)
        throw std::invalid_argument("invalid initialization window or iteration budget");
    positive(config.sample_size, "initialization.sample_size");
    positive(config.fallback_sample_size, "initialization.fallback_sample_size");
    positive(config.minimum_directional_support, "initialization.minimum_directional_support");
    if (config.fallback_sample_size > config.sample_size)
        throw std::invalid_argument(
            "initialization fine sampling size must not exceed the regular size");
    for (double tolerance : config.relative_cost_tolerance) {
        positive(tolerance, "initialization.relative_cost_tolerance");
        if (tolerance >= 1)
            throw std::invalid_argument(
                "initialization relative cost tolerance must be less than one");
    }
    const auto& registration = config.registration;
    positive(registration.voxel_size, "initialization.registration.voxel_size");
    positive(registration.plane_residual_tolerance,
             "initialization.registration.plane_residual_tolerance");
    positive(registration.prior_covariance_scale,
             "initialization.registration.prior_covariance_scale");
    positive(registration.rotation_tolerance, "initialization.registration.rotation_tolerance");
    positive(registration.position_tolerance, "initialization.registration.position_tolerance");
    positive(registration.gyro_variance, "initialization.registration.gyro_variance");
    positive(registration.accel_variance, "initialization.registration.accel_variance");
    positive(registration.bias_variance, "initialization.registration.bias_variance");
    positive(registration.initial_state_variance,
             "initialization.registration.initial_state_variance");
    positive(registration.initial_bias_variance,
             "initialization.registration.initial_bias_variance");
    if (registration.neighbors < 3 || registration.maximum_iterations < 1 ||
        registration.minimum_map_points < static_cast<std::size_t>(registration.neighbors))
        throw std::invalid_argument(
            "invalid coarse initialization registration point count or iteration budget");
}

/// 校验启动几何与优化资源设置。
void validateStartupOptimization(const StartupOptimizationConfig& config) {
    positive(config.voxel_size, "initialization.optimization.voxel_size");
    positive(config.imu_weight, "initialization.optimization.imu_weight");
    positive(config.plane_variance[0], "initialization.optimization.plane_variance[0]");
    positive(config.plane_variance[1], "initialization.optimization.plane_variance[1]");
    positive(config.plane_eigenvalue_ratio, "initialization.optimization.plane_eigenvalue_ratio");
    positive(config.plane_middle_ratio, "initialization.optimization.plane_middle_ratio");
    validateImuNoise(config.noise, "initialization.optimization");
    if (config.maximum_depth < 0 || config.maximum_depth > 8 || config.thread_count < 1 ||
        config.maximum_iterations < 1 || config.minimum_points < 3 || config.minimum_planes < 1 ||
        config.plane_variance[1] > config.plane_variance[0] || config.plane_eigenvalue_ratio >= 1 ||
        config.plane_middle_ratio >= 1)
        throw std::invalid_argument("invalid initialization plane or solver resource parameters");
}

/// 校验观测间隔的数值下限及上下界关系。
void validateWindow(const WindowConfig& window) {
    positive(window.maximum_interval, "window.maximum_interval");
    positive(window.minimum_interval, "window.minimum_interval");
    positive(window.normalized_error, "window.normalized_error");
    if (window.minimum_interval < 1e-9 || 2 * window.minimum_interval > window.maximum_interval)
        throw std::invalid_argument(
            "intervals must satisfy 1 ns <= minimum_interval and 2*minimum_interval <= "
            "maximum_interval");
}
/// 校验传感器几何、重力和启动先验。
void validateInitialConditions(const Config& config) {
    if (!std::isfinite(config.gravity) || config.gravity < 9.6 || config.gravity > 10.0)
        throw std::invalid_argument("sensor.gravity must be within [9.6, 10.0] m/s^2");
    if (!config.initial_accel_bias.allFinite())
        throw std::invalid_argument("invalid initial acceleration bias");
    for (int axis = 0; axis < 3; ++axis) {
        const double deviation = config.initial_accel_bias_std[axis];
        positive(deviation, "initialization.accel_bias_std");
        positive(1 / (deviation * deviation), "acceleration bias prior information");
    }
    positive(config.maximum_range, "sensor.maximum_range");
    if (!std::isfinite(config.minimum_range) || config.minimum_range < 0 ||
        config.minimum_range >= config.maximum_range)
        throw std::invalid_argument("invalid range limits");
    if (!config.body_from_lidar.position.allFinite() ||
        !config.body_from_lidar.rotation.coeffs().allFinite() ||
        std::abs(config.body_from_lidar.rotation.norm() - 1) > .01)
        throw std::invalid_argument("invalid rigid extrinsics");
    for (const Vec3& variance : config.initial_uncertainty.variance)
        for (int axis = 0; axis < 3; ++axis)
            positive(variance[axis], "variance component in initialization.uncertainty");
}

/// 校验惯性噪声、线程数及迭代预算。
void validateFilter(const Config& config) {
    validateImuNoise(config.noise, "filter");
    if (config.maximum_iterations < 1)
        throw std::invalid_argument("filter iteration count must be positive");
    if (config.association_threads < 1 || config.association_threads > 2)
        throw std::invalid_argument("point-to-plane association thread count must be one or two");
    positive(config.convergence_tolerance, "filter.convergence_tolerance");
}

/// 校验地图平面判据、测量噪声和资源上限。
void validateMap(const MapConfig& map) {
    positive(map.voxel_size, "map.voxel_size");
    positive(map.range_sigma, "map.range_sigma");
    positive(map.angle_sigma_degrees, "map.angle_sigma_degrees");
    const float resolution = static_cast<float>(map.voxel_size);
    if (!std::isnormal(resolution) || !std::isnormal(1.f / resolution))
        throw std::invalid_argument("map scale must be representable as a valid float");
    if (map.maximum_roots == 0)
        throw std::invalid_argument("map root voxel capacity must be positive");
}
}  // namespace

/// 校验配置并规范化外参旋转。
Config normalizeConfig(Config config) {
    validateConfig(config);
    // 将允许范围内的外参四元数归一化。
    if (std::abs(config.body_from_lidar.rotation.squaredNorm() - 1) >
        4 * std::numeric_limits<double>::epsilon())
        config.body_from_lidar.rotation.normalize();
    return config;
}

/// 校验算法参数、坐标变换与资源设置。
void validateConfig(const Config& config) {
    validateInitialConditions(config);
    validateFilter(config);
    validateMap(config.map);
    validateSampling(config.sampling);
    validateWindow(config.window);
    validateInitialization(config.initialization);
    validateStartupOptimization(config.initialization.optimization);
}
}  // namespace zlio
