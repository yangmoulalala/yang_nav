/**
 * @file body_extrinsic.hpp
 * @brief 按消息时间戳从 TF 查询 sensor <- body 外参，支持云台等可动关节。
 */
#pragma once

#include <tf2/buffer_core.h>

#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace tf2_ros {
class Buffer;
class TransformListener;
}  // namespace tf2_ros

namespace backend::ros {

/// 不依赖节点的时间索引外参解析逻辑，线程安全。
/// 查询规则：优先使用请求时刻的精确变换；不可用时使用 TF 中最新的变换，
/// 其时间戳与请求时刻相差不超过 tolerance 时视为有效；否则节流告警并沿用上一次有效值。
class BodyExtrinsicResolver {
public:
    /// 本次结果的来源。
    enum class Source { Exact, Latest, Stale };
    /// 一次查询的结果：T_sensor_body 及其来源。
    struct Result {
        Eigen::Matrix4d T_sensor_body = Eigen::Matrix4d::Identity();
        Source source = Source::Stale;
    };
    using LogFunction = std::function<void(const std::string&)>;

    /// 绑定 TF 缓存、坐标系对（target=sensor_frame，source=body_frame）、容差（s）与日志回调。
    BodyExtrinsicResolver(const tf2::BufferCore& buffer, std::string sensor_frame,
                          std::string body_frame, double tolerance_seconds, LogFunction warn,
                          LogFunction info);

    /// 查询最新可用变换并记为有效值；尚不可用时返回空并写出原因。
    std::optional<Eigen::Matrix4d> tryInitialize(std::string* error);
    /// 返回请求时刻（s，非正值表示最新）的 T_sensor_body；必须先初始化成功。
    Result resolve(double stamp_seconds);
    /// 便捷接口：只返回外参矩阵。
    Eigen::Matrix4d at(double stamp_seconds) { return resolve(stamp_seconds).T_sensor_body; }
    /// 返回最近一次有效外参。
    Eigen::Matrix4d lastGood() const;
    /// 返回是否已获得过有效外参。
    bool initialized() const;

private:
    /// 记录有效值；与上次报告相比旋转或平移明显变化时节流输出信息日志。
    void acceptLocked(const Eigen::Matrix4d& T_sensor_body);
    /// 以稳态时钟节流告警。
    void warnThrottledLocked(const std::string& message);

    const tf2::BufferCore& buffer_;
    const std::string sensor_frame_;
    const std::string body_frame_;
    const double tolerance_seconds_;
    LogFunction warn_;
    LogFunction info_;
    mutable std::mutex mutex_;
    std::optional<Eigen::Matrix4d> last_good_;
    Eigen::Matrix4d last_reported_ = Eigen::Matrix4d::Identity();
    std::chrono::steady_clock::time_point last_warn_{};
    std::chrono::steady_clock::time_point last_info_{};
    bool warned_ = false;
    bool reported_ = false;
};

/// 拥有 tf2_ros Buffer 与 TransformListener，按时间戳提供 T_sensor_body。
class BodyExtrinsicProvider {
public:
    /// 创建 TF 缓存和带独立 spin 线程的监听器；节点构造期间无需执行器即可接收 /tf。
    BodyExtrinsicProvider(rclcpp::Node& node, const std::string& sensor_frame,
                          const std::string& body_frame, double tolerance_seconds);
    ~BodyExtrinsicProvider();
    BodyExtrinsicProvider(const BodyExtrinsicProvider&) = delete;
    BodyExtrinsicProvider& operator=(const BodyExtrinsicProvider&) = delete;

    /// 阻塞等待首个可用外参（rclcpp::ok() 期间重试并节流告警），返回启动时外参。
    Eigen::Matrix4d waitForInitial();
    /// 返回请求时刻（s）的 T_sensor_body，规则见 BodyExtrinsicResolver。
    Eigen::Matrix4d at(double stamp_seconds) { return resolver_->at(stamp_seconds); }

private:
    rclcpp::Node& node_;
    std::string sensor_frame_;
    std::string body_frame_;
    std::unique_ptr<tf2_ros::Buffer> buffer_;
    std::unique_ptr<tf2_ros::TransformListener> listener_;
    std::unique_ptr<BodyExtrinsicResolver> resolver_;
};

}  // namespace backend::ros
