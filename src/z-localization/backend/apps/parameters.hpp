/**
 * @file parameters.hpp
 * @brief 从 ROS 参数树读取并校验后端有效配置。
 */
#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <vector>

#include "apps/config.hpp"

namespace backend::ros {

/// 按输入参数（sensor_frame、body_frame、容差）建立外参来源，阻塞等待并返回启动时的
/// T_sensor_body，即 body 在传感器系中的位姿。
using BodyExtrinsicLookup = std::function<Eigen::Matrix4d(const RosInputParameters& input)>;

/// 负责参数读取、类型兼容和统一 schema 校验。
class ParameterReader {
public:
    /// 绑定参数读取和日志所使用的 ROS 节点。
    explicit ParameterReader(rclcpp::Node& node) : node_(node) {}

    /// 加载并校验完整参数树；缺失、未知或已废弃参数会抛出异常。
    /// 参数全部通过校验后才调用 lookup 获取 body 外参，再派生依赖外参的配置。
    NodeConfig load(const BodyExtrinsicLookup& lookup);

private:
    /// 读取输入传输与坐标系名称。
    void loadCommonInput();
    /// 以启动时 T_sensor_body 派生启动提示位姿及 UWB 外参的回退值。
    void applyBodyExtrinsic(const Eigen::Matrix4d& T_sensor_body);
    /// 读取地图、关键帧和统一配准参数。
    void loadMapTrajectoryAndRegistration();
    /// 读取连续跟踪与联合创新参数。
    void loadTracking();
    /// 读取状态机恢复路径参数。
    void loadRecovery();
    /// 读取 UWB 测量、外参和在线标定参数。
    void loadUwb();
    /// 读取在线坐标标定开关及固定 UWB 坐标变换。
    void loadUwbFrameCalibration();
    /// 读取天线杆臂及位置测量噪声模型。
    void loadUwbMeasurementModel();
    /// 读取 UWB 启动对齐的配对与稳定性参数。
    void loadUwbBootstrap();
    /// 读取回环与因子图参数。
    void loadLoopAndGraph();
    /// 读取图先验、过程噪声、鲁棒核及固定窗口参数。
    void loadGraph();
    /// 读取 reset 事务和启动提示参数。
    void loadResetAndStartup();
    /// 读取可视化、服务和世界点云输出参数。
    void loadOutputs();
    /// 读取所选地点描述子的参数。
    void loadDescriptor();
    /// 校验跨参数能力组合并执行已声明覆盖。
    void validateRelationships();
    /// 拒绝未被 schema 消费的未知参数。
    void validateParameterSchema() const;

    /// 读取一个必填参数并记录其 schema 所有权；兼容整数和浮点 YAML 写法。
    /// 节点按覆盖值自动声明参数，因此未由参数文件或 launch 提供即视为缺失，不使用任何代码默认值。
    template <typename T>
    void getParam(const std::string& path, T& value) {
        std::string name = path;
        std::replace(name.begin(), name.end(), '/', '.');
        consumed_parameter_names_.insert(name);
        if (!node_.has_parameter(name)) {
            throw std::invalid_argument("missing required backend parameter: " + name);
        }
        const rclcpp::Parameter parameter = node_.get_parameter(name);
        if constexpr (std::is_same_v<T, double>) {
            value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                        ? static_cast<double>(parameter.as_int())
                        : parameter.as_double();
        } else if constexpr (std::is_same_v<T, int>) {
            value = parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE
                        ? static_cast<int>(parameter.as_double())
                        : static_cast<int>(parameter.as_int());
        } else if constexpr (std::is_same_v<T, std::vector<double>>) {
            if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
                const auto& integers = parameter.as_integer_array();
                value.assign(integers.begin(), integers.end());
            } else {
                value = parameter.as_double_array();
            }
        } else {
            value = parameter.get_value<T>();
        }
    }

    /// 读取一个必填参数并按值返回；用于需要换算单位的参数，避免写出看似默认值、实际从不生效的局部初值。
    template <typename T>
    T param(const std::string& path) {
        T value{};
        getParam(path, value);
        return value;
    }

    rclcpp::Node& node_;
    NodeConfig parameters_;
    /// 启用时的 epoch 0 启动提示 T_map_body，用启动时外参换算为 T_map_sensor。
    std::optional<Eigen::Matrix4d> startup_T_map_body_;
    std::unordered_set<std::string> consumed_parameter_names_;
};

}  // namespace backend::ros
