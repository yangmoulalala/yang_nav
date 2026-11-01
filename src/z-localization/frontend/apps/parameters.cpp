/**
 * @file parameters.cpp
 * @brief 从 ROS 参数文件读取里程计的算法与传输参数。
 */
#include "parameters.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "parameter_reader.hpp"

namespace zlio::ros {
namespace {

/// 读取三轴向量参数。
Vec3 readVector(ParameterReader& reader, const std::string& name) {
    const auto values = reader.reals(name, 3);
    return Vec3(values[0], values[1], values[2]);
}

/// 读取惯性采样方差及两类偏置随机游走强度。
void readImuNoise(ParameterReader& reader, const std::string& prefix, ImuNoise& noise) {
    noise.gyro_sample_variance = reader.real(prefix + ".gyro_variance");
    noise.accel_sample_variance = reader.real(prefix + ".accel_variance");
    noise.gyro_bias_psd = reader.real(prefix + ".gyro_bias_psd");
    noise.accel_bias_psd = reader.real(prefix + ".accel_bias_psd");
}

/// 读取重力与测距范围；外参改由 TF 提供。
void readSensor(ParameterReader& reader, Config& config) {
    config.gravity = reader.real("sensor.gravity");
    config.minimum_range = reader.real("sensor.minimum_range");
    config.maximum_range = reader.real("sensor.maximum_range");
}

/// 读取启动粗配准参数。
void readRegistration(ParameterReader& reader, RegistrationConfig& config) {
    const std::string p = "initialization.registration.";
    config.voxel_size = reader.real(p + "voxel_size");
    config.neighbors = static_cast<int>(reader.integer(p + "neighbors"));
    config.minimum_map_points = reader.size(p + "minimum_map_points");
    config.maximum_iterations = static_cast<int>(reader.integer(p + "maximum_iterations"));
    config.plane_residual_tolerance = reader.real(p + "plane_residual_tolerance");
    config.prior_covariance_scale = reader.real(p + "prior_covariance_scale");
    config.rotation_tolerance = reader.real(p + "rotation_tolerance");
    config.position_tolerance = reader.real(p + "position_tolerance");
    config.gyro_variance = reader.real(p + "gyro_variance");
    config.accel_variance = reader.real(p + "accel_variance");
    config.bias_variance = reader.real(p + "bias_variance");
    config.initial_state_variance = reader.real(p + "initial_state_variance");
    config.initial_bias_variance = reader.real(p + "initial_bias_variance");
}

/// 读取启动窗口优化参数。
void readStartupOptimization(ParameterReader& reader, StartupOptimizationConfig& config) {
    const std::string p = "initialization.optimization";
    config.voxel_size = reader.real(p + ".voxel_size");
    config.maximum_depth = static_cast<int>(reader.integer(p + ".maximum_depth"));
    config.minimum_points = reader.size(p + ".minimum_points");
    config.minimum_planes = reader.size(p + ".minimum_planes");
    const auto variance = reader.reals(p + ".plane_variance", 2);
    config.plane_variance = {variance[0], variance[1]};
    config.plane_eigenvalue_ratio = reader.real(p + ".plane_eigenvalue_ratio");
    config.plane_middle_ratio = reader.real(p + ".plane_middle_ratio");
    config.imu_weight = reader.real(p + ".imu_weight");
    config.thread_count = static_cast<int>(reader.integer(p + ".thread_count"));
    config.maximum_iterations = static_cast<int>(reader.integer(p + ".maximum_iterations"));
    readImuNoise(reader, p, config.noise);
}

/// 读取启动窗口、偏置先验和初始不确定度。
void readInitialization(ParameterReader& reader, Config& config) {
    const std::string p = "initialization.";
    auto& init = config.initialization;
    config.initial_accel_bias = readVector(reader, p + "accel_bias");
    config.initial_accel_bias_std = readVector(reader, p + "accel_bias_std");
    init.window_frames = reader.size(p + "window_frames");
    init.seed_imu_samples = reader.size(p + "seed_imu_samples");
    init.maximum_refinements = static_cast<int>(reader.integer(p + "maximum_refinements"));
    init.sample_size = reader.real(p + "sample_size");
    init.fallback_sample_size = reader.real(p + "fallback_sample_size");
    init.minimum_sampled_points = reader.size(p + "minimum_sampled_points");
    init.minimum_directional_support = reader.real(p + "minimum_directional_support");
    const auto tolerance = reader.reals(p + "relative_cost_tolerance", 2);
    init.relative_cost_tolerance = {tolerance[0], tolerance[1]};
    readRegistration(reader, init.registration);
    readStartupOptimization(reader, init.optimization);
    config.initial_uncertainty.allow_configured_fallback =
        reader.boolean(p + "uncertainty.allow_configured_fallback");
    const auto variance = reader.reals(p + "uncertainty.state_variance", 5);
    for (std::size_t i = 0; i < variance.size(); ++i)
        config.initial_uncertainty.variance[i].setConstant(variance[i]);
}

/// 读取运行滤波参数。
void readFilter(ParameterReader& reader, Config& config) {
    config.association_threads = static_cast<int>(reader.integer("filter.association_threads"));
    config.maximum_iterations = static_cast<int>(reader.integer("filter.maximum_iterations"));
    config.convergence_tolerance = reader.real("filter.convergence_tolerance");
    readImuNoise(reader, "filter", config.noise);
}

/// 读取概率地图参数。
void readMap(ParameterReader& reader, MapConfig& map) {
    map.voxel_size = reader.real("map.voxel_size");
    map.maximum_roots = reader.size("map.maximum_roots");
    map.range_sigma = reader.real("map.range_sigma");
    map.angle_sigma_degrees = reader.real("map.angle_sigma_degrees");
}

/// 读取自适应采样参数。
void readSampling(ParameterReader& reader, SamplingConfig& s) {
    s.initial_size = reader.real("sampling.initial_size");
    s.minimum_size = reader.real("sampling.minimum_size");
    s.maximum_size = reader.real("sampling.maximum_size");
    s.history_size = reader.size("sampling.history_size");
    s.minimum_points = reader.real("sampling.minimum_points");
    s.maximum_points = reader.real("sampling.maximum_points");
    s.scale_threshold = reader.real("sampling.scale_threshold");
    s.exponent = reader.real("sampling.exponent");
    s.proportional_scale = reader.real("sampling.proportional_scale");
    s.derivative_scale = reader.real("sampling.derivative_scale");
    s.proportional_minimum = reader.real("sampling.proportional_minimum");
    s.proportional_maximum = reader.real("sampling.proportional_maximum");
    s.derivative_minimum = reader.real("sampling.derivative_minimum");
    s.derivative_maximum = reader.real("sampling.derivative_maximum");
}

/// 读取观测间隔调度参数。
void readWindow(ParameterReader& reader, WindowConfig& window) {
    window.maximum_interval = reader.real("window.maximum_interval");
    window.minimum_interval = reader.real("window.minimum_interval");
    window.normalized_error = reader.real("window.normalized_error");
}

/// 读取坐标系、输入解码、缓存容量、输出与会话参数。
void readTransport(ParameterReader& reader, NodeParameters& parameters) {
    parameters.frames.world = reader.text("frames.world");
    parameters.frames.body = reader.text("frames.imu");
    parameters.lidar_frame = reader.text("frames.lidar");
    if (parameters.frames.world.empty() || parameters.frames.body.empty() ||
        parameters.lidar_frame.empty())
        throw std::invalid_argument("frames.world, frames.imu and frames.lidar must not be empty");

    parameters.input_type = reader.text("input.type");
    if (parameters.input_type != "pointcloud2" && parameters.input_type != "livox")
        throw std::invalid_argument("input.type must be pointcloud2 or livox");
    parameters.cloud_format.time_field = reader.text("input.point_time_field");
    parameters.cloud_format.time_scale = reader.real("input.point_time_scale");
    parameters.cloud_format.absolute_time = reader.boolean("input.point_time_absolute");
    parameters.livox_timebase = reader.boolean("input.livox_use_timebase");
    parameters.imu_acceleration_scale = reader.real("input.imu_acceleration_scale");
    if (!std::isfinite(parameters.imu_acceleration_scale) || parameters.imu_acceleration_scale <= 0)
        throw std::invalid_argument("input.imu_acceleration_scale must be finite and positive");
    auto& history = parameters.input_limits.imu_history;
    history.maximum_duration = reader.real("input.imu_history_seconds");
    history.maximum_interval = reader.real("input.maximum_imu_interval");
    history.maximum_samples = reader.count("input.maximum_imu_samples");
    parameters.input_limits.maximum_lidar_points = reader.count("input.maximum_buffered_points");
    parameters.cloud_format.maximum_points = parameters.input_limits.maximum_lidar_points;
    parameters.maximum_queue_events = reader.count("input.maximum_queue_events");

    parameters.output_resolution = reader.real("output.resolution");
    parameters.publish_tf = reader.boolean("output.publish_tf");
    parameters.base_frame = reader.text("frames.base");
    if (parameters.base_frame.empty())
        throw std::invalid_argument("frames.base must not be empty");
    parameters.base_extrinsic_tolerance = reader.real("output.base_extrinsic_tolerance");
    if (!std::isfinite(parameters.base_extrinsic_tolerance) ||
        parameters.base_extrinsic_tolerance < 0)
        throw std::invalid_argument("output.base_extrinsic_tolerance must be finite and >= 0");

    parameters.wait_for_start = reader.boolean("session.wait_for_start");
    parameters.reset_mode = reader.text("session.reset_mode");
    if (parameters.reset_mode != "warm" && parameters.reset_mode != "cold")
        throw std::invalid_argument("session.reset_mode must be warm or cold");
    parameters.coordinated_reset = reader.boolean("session.coordinated_reset");
}

}  // namespace

/// 读取全部必填参数，并在返回前拒绝未知参数。
NodeParameters loadNodeParameters(rclcpp::Node& node) {
    ParameterReader reader(node);
    NodeParameters parameters;
    readSensor(reader, parameters.config);
    readInitialization(reader, parameters.config);
    readFilter(reader, parameters.config);
    readMap(reader, parameters.config.map);
    readSampling(reader, parameters.config.sampling);
    readWindow(reader, parameters.config.window);
    readTransport(reader, parameters);
    reader.rejectUnknown();
    return parameters;
}

}  // namespace zlio::ros
