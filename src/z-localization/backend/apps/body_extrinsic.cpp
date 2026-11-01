/**
 * @file body_extrinsic.cpp
 * @brief 按时间戳查询 sensor <- body 外参，精确时刻不可用时回退到最新或上次有效值。
 */

#include "apps/body_extrinsic.hpp"

#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

#include <tf2_eigen/tf2_eigen.hpp>

#include "backend/math.hpp"

namespace backend::ros {
namespace {
constexpr auto kWarnPeriod = std::chrono::seconds(5);
constexpr auto kInfoPeriod = std::chrono::seconds(1);
/// 外参变化超过该旋转角或平移时输出一次信息日志，便于确认云台转动已被跟踪。
constexpr double kReportRotationRad = 2.0 * M_PI / 180.0;
constexpr double kReportTranslationM = 0.01;

double stampSeconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

/// 把外参格式化为平移与 rpy（度）。
std::string describe(const Eigen::Matrix4d& T) {
    const Eigen::Vector3d rpy = rpyFromRotation(T.block<3, 3>(0, 0)) * 180.0 / M_PI;
    std::ostringstream text;
    text << "t_sensor_body=[" << T(0, 3) << ", " << T(1, 3) << ", " << T(2, 3)
         << "] m, rpy_sensor_body=[" << rpy.x() << ", " << rpy.y() << ", " << rpy.z() << "] deg";
    return text.str();
}
}  // namespace

BodyExtrinsicResolver::BodyExtrinsicResolver(const tf2::BufferCore& buffer,
                                             std::string sensor_frame, std::string body_frame,
                                             const double tolerance_seconds, LogFunction warn,
                                             LogFunction info)
    : buffer_(buffer),
      sensor_frame_(std::move(sensor_frame)),
      body_frame_(std::move(body_frame)),
      tolerance_seconds_(tolerance_seconds),
      warn_(std::move(warn)),
      info_(std::move(info)) {
    if (!std::isfinite(tolerance_seconds_) || tolerance_seconds_ < 0.0)
        throw std::invalid_argument("body extrinsic tolerance must be finite and non-negative");
}

std::optional<Eigen::Matrix4d> BodyExtrinsicResolver::tryInitialize(std::string* error) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!buffer_.canTransform(sensor_frame_, body_frame_, tf2::TimePointZero, error))
        return std::nullopt;
    const Eigen::Matrix4d T =
        tf2::transformToEigen(
            buffer_.lookupTransform(sensor_frame_, body_frame_, tf2::TimePointZero))
            .matrix();
    if (!T.allFinite()) {
        if (error != nullptr)
            *error = "body extrinsic from TF contains a non-finite value";
        return std::nullopt;
    }
    acceptLocked(T);
    return T;
}

BodyExtrinsicResolver::Result BodyExtrinsicResolver::resolve(const double stamp_seconds) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!last_good_)
        throw std::logic_error("body extrinsic queried before initialization");
    Result result;
    try {
        // 1. 请求时刻的精确变换（含插值；全静态链对任意时刻均可用）。
        if (std::isfinite(stamp_seconds) && stamp_seconds > 0.0) {
            const tf2::TimePoint time = tf2::timeFromSec(stamp_seconds);
            if (buffer_.canTransform(sensor_frame_, body_frame_, time, nullptr)) {
                const Eigen::Matrix4d T =
                    tf2::transformToEigen(buffer_.lookupTransform(sensor_frame_, body_frame_, time))
                        .matrix();
                if (T.allFinite()) {
                    acceptLocked(T);
                    result.T_sensor_body = T;
                    result.source = Source::Exact;
                    return result;
                }
            }
        }
        // 2. 最新可用变换；时间戳为 0 表示全静态链，始终有效。
        std::string error;
        if (buffer_.canTransform(sensor_frame_, body_frame_, tf2::TimePointZero, &error)) {
            const auto transform =
                buffer_.lookupTransform(sensor_frame_, body_frame_, tf2::TimePointZero);
            const Eigen::Matrix4d T = tf2::transformToEigen(transform).matrix();
            const double latest = stampSeconds(transform.header.stamp);
            const bool timeless = latest <= 0.0 || !std::isfinite(stamp_seconds) ||
                                  stamp_seconds <= 0.0;
            const double age = timeless ? 0.0 : std::abs(stamp_seconds - latest);
            if (T.allFinite() && age <= tolerance_seconds_) {
                acceptLocked(T);
                result.T_sensor_body = T;
                result.source = Source::Latest;
                return result;
            }
            std::ostringstream text;
            text << "body extrinsic TF " << sensor_frame_ << " <- " << body_frame_
                 << " is stale: latest stamp " << latest << " s differs from requested "
                 << stamp_seconds << " s by " << age << " s > common.body_extrinsic_tolerance="
                 << tolerance_seconds_ << " s; keeping the last good extrinsic";
            warnThrottledLocked(text.str());
        } else {
            warnThrottledLocked("body extrinsic TF " + sensor_frame_ + " <- " + body_frame_ +
                                " unavailable (" + error +
                                "); keeping the last good extrinsic");
        }
    } catch (const tf2::TransformException& exception) {
        warnThrottledLocked(std::string("body extrinsic TF lookup failed (") + exception.what() +
                            "); keeping the last good extrinsic");
    }
    result.T_sensor_body = *last_good_;
    result.source = Source::Stale;
    return result;
}

Eigen::Matrix4d BodyExtrinsicResolver::lastGood() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_good_.value_or(Eigen::Matrix4d::Identity());
}

bool BodyExtrinsicResolver::initialized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_good_.has_value();
}

void BodyExtrinsicResolver::acceptLocked(const Eigen::Matrix4d& T_sensor_body) {
    last_good_ = T_sensor_body;
    const Eigen::Matrix4d delta = last_reported_.inverse() * T_sensor_body;
    const double angle = Eigen::AngleAxisd(Eigen::Matrix3d(delta.block<3, 3>(0, 0))).angle();
    const bool changed = !reported_ || std::abs(angle) > kReportRotationRad ||
                         delta.block<3, 1>(0, 3).norm() > kReportTranslationM;
    const auto now = std::chrono::steady_clock::now();
    if (!changed || (reported_ && now - last_info_ < kInfoPeriod))
        return;
    if (info_)
        info_(std::string(reported_ ? "body extrinsic changed " : "body extrinsic ") +
              "from TF " + sensor_frame_ + " <- " + body_frame_ + ": " +
              describe(T_sensor_body));
    reported_ = true;
    last_reported_ = T_sensor_body;
    last_info_ = now;
}

void BodyExtrinsicResolver::warnThrottledLocked(const std::string& message) {
    const auto now = std::chrono::steady_clock::now();
    if (warned_ && now - last_warn_ < kWarnPeriod)
        return;
    warned_ = true;
    last_warn_ = now;
    if (warn_)
        warn_(message);
}

BodyExtrinsicProvider::BodyExtrinsicProvider(rclcpp::Node& node, const std::string& sensor_frame,
                                             const std::string& body_frame,
                                             const double tolerance_seconds)
    : node_(node), sensor_frame_(sensor_frame), body_frame_(body_frame) {
    buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    // 监听器使用独立 spin 线程，节点构造期间及运行期间都持续接收 /tf 与 /tf_static。
    listener_ = std::make_unique<tf2_ros::TransformListener>(*buffer_, &node, true);
    auto logger = node.get_logger();
    resolver_ = std::make_unique<BodyExtrinsicResolver>(
        *buffer_, sensor_frame, body_frame, tolerance_seconds,
        [logger](const std::string& message) { RCLCPP_WARN(logger, "%s", message.c_str()); },
        [logger](const std::string& message) { RCLCPP_INFO(logger, "%s", message.c_str()); });
}

BodyExtrinsicProvider::~BodyExtrinsicProvider() = default;

Eigen::Matrix4d BodyExtrinsicProvider::waitForInitial() {
    std::string error;
    while (rclcpp::ok()) {
        if (const auto T = resolver_->tryInitialize(&error))
            return *T;
        RCLCPP_WARN_STREAM_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
                                    "waiting for TF " << sensor_frame_ << " <- " << body_frame_
                                                      << " (body extrinsic): " << error);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    throw std::runtime_error("shutdown requested while waiting for body extrinsic TF " +
                             sensor_frame_ + " <- " + body_frame_);
}

}  // namespace backend::ros
