/**
 * @file node_setup.cpp
 * @brief 装配后端模型、ROS 端口、订阅及周期任务。
 */

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "apps/body_extrinsic.hpp"
#include "apps/config.hpp"
#include "apps/node.hpp"
#include "apps/parameters.hpp"
#include "backend/math.hpp"

namespace backend::ros {
namespace {
/// 按配置构造对应的地点描述子。
std::unique_ptr<PlaceDescriptor> makePlaceDescriptor(const DescriptorParameters& parameters) {
    if (parameters.type == "scan_context") {
        auto descriptor = std::make_unique<ScanContextDescriptor>();
        descriptor->configure(parameters.scan_context);
        return descriptor;
    }
    if (parameters.type == "gprobe") {
        auto descriptor = std::make_unique<GProbeDescriptor>();
        descriptor->configure(parameters.gprobe);
        return descriptor;
    }
    throw std::invalid_argument("common.descriptor_type must be scan_context or gprobe");
}

}  // namespace

/// 构造后端节点并依次安装运行时、ROS 端口及周期任务。
LocalizationNode::LocalizationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("backend_node", options),
      frontend_(
          *this, failed_,
          [this](const char* name, std::exception_ptr error) { handleFailure(name, error); },
          [this] { onFrontendResetBoundary(); }),
      publisher_(
          *this, parameters_, trajectory_, prior_map_, failed_,
          [this](const char* name, std::exception_ptr error) { handleFailure(name, error); }) {
    initializeRuntime();
    publisher_.configure();
    configureExecution();
    configureServices();
    configureTimers();
    completeStartup();
}

/// 将适配组件捕获的异常交回进程统一失败分类。
void LocalizationNode::handleFailure(const char* name, std::exception_ptr error) {
    invoke(name, [&] { std::rethrow_exception(error); });
}

/// 释放节点持有的模型、订阅及发布资源。
LocalizationNode::~LocalizationNode() = default;

/// 执行器退出后，按关闭保存策略完成运行地图写出。
void LocalizationNode::finish() {
    if (failed() || !parameters_.maps.save_on_shutdown || parameters_.maps.save_directory.empty()) {
        return;
    }
    invoke(
        "shutdown map save",
        [&] {
            RCLCPP_INFO_STREAM(get_logger(), "saving runtime trajectory map to "
                                                 << parameters_.maps.save_directory);
            saveTrajectoryMap(parameters_.maps.save_directory);
        },
        FailurePolicy::Skip);
}

/// 读取有效参数并构造描述子、先验地图及运行时控制器。
void LocalizationNode::initializeRuntime() {
    // 外参来源在运行期间持续监听 TF；启动时阻塞到首个外参可用（云台等可动关节随时间跟踪）。
    parameters_ = ParameterReader(*this).load([this](const RosInputParameters& input) {
        body_extrinsic_ = std::make_unique<BodyExtrinsicProvider>(
            *this, input.sensor_frame, input.body_frame, input.body_extrinsic_tolerance);
        return body_extrinsic_->waitForInitial();
    });
    publisher_.bindBodyExtrinsic(body_extrinsic_.get());
    descriptor_ = makePlaceDescriptor(parameters_.descriptor);
    EffectiveConfig effective_config = resolveConfig(parameters_.runtime, descriptor_->enabled());
    parameters_.runtime = effective_config.values;

    if (parameters_.runtime.prior_map_enabled &&
        parameters_.runtime.graph.fixed_lag_seconds <= 0.0) {
        RCLCPP_WARN(get_logger(),
                    "prior-map localization is using an unbounded pose graph; set "
                    "graph.fixed_lag_seconds>0 for long-running deployment");
    }
    if (parameters_.runtime.prior_map_enabled && !loadPriorMap(parameters_.maps.prior_directory)) {
        throw std::runtime_error("failed to load immutable prior map");
    }

    controller_ = std::make_unique<Localization>(trajectory_, prior_map_, *descriptor_,
                                                 std::move(effective_config), steady_clock_,
                                                 frontend_, publisher_, publisher_);
    frontend_.configure(parameters_.input.frontend == "zlio", parameters_.services.frontend_reset);
    publisher_.bind(*controller_);
    controller_->initialize();
    frontend_.bind(*controller_);
}

/// 创建互斥回调组并连接当前前端代次的输入订阅。
void LocalizationNode::configureExecution() {
    high_rate_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    maintenance_callback_group_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    map_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    configureInputs();
}

/// 创建前端重置客户端及地图保存、重定位和系统重置服务。
void LocalizationNode::configureServices() {
    frontend_.connect();
    save_service_ = create_service<std_srvs::srv::Trigger>(
        parameters_.services.save_map,
        std::bind(&LocalizationNode::saveMapService, this, std::placeholders::_1,
                  std::placeholders::_2),
        rclcpp::ServicesQoS(), map_callback_group_);
    relocalize_service_ = create_service<std_srvs::srv::Trigger>(
        parameters_.services.relocalize, std::bind(&LocalizationNode::relocalizeService, this,
                                                   std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
        parameters_.services.reset, std::bind(&LocalizationNode::resetService, this,
                                              std::placeholders::_1, std::placeholders::_2));
}

/// 安装维护周期及按需启用的地图发布周期。
void LocalizationNode::configureTimers() {
    maintenance_timer_ = create_wall_timer(
        std::chrono::duration<double>(parameters_.input.maintenance_period),
        std::bind(&LocalizationNode::maintenanceCallback, this), maintenance_callback_group_);
    if (publisher_.runtimeMapEnabled()) {
        map_timer_ = create_wall_timer(
            std::chrono::duration<double>(std::max(0.1, parameters_.maps.publish_period)),
            std::bind(&LocalizationNode::mapPublishCallback, this), map_callback_group_);
    }
}

/// 发布静态地图、坐标轴和初始状态并报告有效能力。
void LocalizationNode::completeStartup() {
    publisher_.publishPriorMap();
    publisher_.publishMapAxes();
    publisher_.publishState(controller_->snapshot());
    RCLCPP_INFO_STREAM(
        get_logger(),
        "unified backend ready: prior_map="
            << (parameters_.runtime.prior_map_enabled ? "enabled" : "disabled")
            << ", loop=" << (parameters_.runtime.loop.enabled ? "enabled" : "disabled")
            << ", frontend=" << parameters_.input.frontend
            << (parameters_.input.frontend == "zlio"
                    ? std::string()
                    : ", odom=" + parameters_.input.odom_topic +
                          ", sync_odom=" + parameters_.input.sync_odom_topic +
                          ", cloud=" + parameters_.input.cloud_topic));
}

}  // namespace backend::ros
