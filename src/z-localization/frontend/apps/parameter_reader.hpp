/**
 * @file parameter_reader.hpp
 * @brief 从 ROS 参数文件读取必填参数，并拒绝缺失、类型错误或未知的参数。
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace zlio::ros {

/// 返回开启参数自动声明的节点选项；参数文件是节点参数的唯一来源。
inline rclcpp::NodeOptions parameterFileOptions() {
    return rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
}

/// 读取参数文件提供的必填参数，记录已消费名称以便拒绝拼写错误。
class ParameterReader {
public:
    /// 绑定已按参数文件自动声明参数的节点。
    explicit ParameterReader(rclcpp::Node& node) : node_(node) {}

    /// 读取浮点参数，兼容整数 YAML 写法。
    double real(const std::string& name) {
        const auto parameter = require(name);
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            return static_cast<double>(parameter.as_int());
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            throw typeError(parameter, "double");
        return parameter.as_double();
    }

    /// 读取整数参数。
    std::int64_t integer(const std::string& name) {
        const auto parameter = require(name);
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
            throw typeError(parameter, "integer");
        return parameter.as_int();
    }

    /// 读取非负整数参数，数值下限由调用方或算法校验负责。
    std::size_t size(const std::string& name) {
        const auto value = integer(name);
        if (value < 0)
            throw std::invalid_argument("parameter '" + name + "' must be non-negative");
        return static_cast<std::size_t>(value);
    }

    /// 读取正整数参数。
    std::size_t count(const std::string& name) {
        const auto value = integer(name);
        if (value <= 0)
            throw std::invalid_argument("parameter '" + name + "' must be positive");
        return static_cast<std::size_t>(value);
    }

    /// 读取布尔参数。
    bool boolean(const std::string& name) {
        const auto parameter = require(name);
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
            throw typeError(parameter, "bool");
        return parameter.as_bool();
    }

    /// 读取字符串参数。
    std::string text(const std::string& name) {
        const auto parameter = require(name);
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
            throw typeError(parameter, "string");
        return parameter.as_string();
    }

    /// 读取固定长度的浮点数组，兼容整数数组写法。
    std::vector<double> reals(const std::string& name, std::size_t length) {
        const auto parameter = require(name);
        std::vector<double> values;
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
            const auto& integers = parameter.as_integer_array();
            values.assign(integers.begin(), integers.end());
        } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            values = parameter.as_double_array();
        } else {
            throw typeError(parameter, "double array");
        }
        if (values.size() != length)
            throw std::invalid_argument("parameter '" + name + "' must have exactly " +
                                        std::to_string(length) + " elements");
        return values;
    }

    /// 拒绝参数文件中未被读取的参数，框架参数除外。
    void rejectUnknown() const {
        const auto listed = node_.list_parameters({}, 0);
        std::vector<std::string> unknown;
        for (const auto& name : listed.names)
            if (consumed_.count(name) == 0 && !isFrameworkParameter(name))
                unknown.push_back(name);
        if (unknown.empty())
            return;
        std::ostringstream message;
        message << "unknown parameters for node " << node_.get_fully_qualified_name()
                << " (check spelling against the parameter documentation): ";
        for (std::size_t index = 0; index < unknown.size(); ++index)
            message << (index == 0 ? "" : ", ") << unknown[index];
        throw std::invalid_argument(message.str());
    }

private:
    /// 识别归属 rclcpp 与 QoS 覆盖机制的框架参数。
    static bool isFrameworkParameter(const std::string& name) {
        return name == "use_sim_time" || name == "start_type_description_service" ||
               name.rfind("qos_overrides.", 0) == 0;
    }

    /// 记录参数归属，缺失时给出明确的必填提示。
    rclcpp::Parameter require(const std::string& name) {
        consumed_.insert(name);
        if (!node_.has_parameter(name))
            throw std::invalid_argument("missing required parameter '" + name + "' for node " +
                                        node_.get_fully_qualified_name() +
                                        "; provide it in the ROS params file");
        const auto parameter = node_.get_parameter(name);
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
            throw std::invalid_argument("missing required parameter '" + name + "' for node " +
                                        node_.get_fully_qualified_name());
        return parameter;
    }

    /// 构造参数类型不匹配的异常。
    static std::invalid_argument typeError(const rclcpp::Parameter& parameter,
                                           const char* expected) {
        return std::invalid_argument("parameter '" + parameter.get_name() + "' must be of type " +
                                     expected + ", got " + parameter.get_type_name());
    }

    rclcpp::Node& node_;
    std::unordered_set<std::string> consumed_;
};

}  // namespace zlio::ros
