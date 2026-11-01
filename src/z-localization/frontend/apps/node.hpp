/**
 * @file node.hpp
 * @brief 声明前端 ROS 节点及其输入、估计和发布资源的所有权。
 */
#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <future>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <localization_interfaces/msg/frontend_frame.hpp>
#include <localization_interfaces/msg/frontend_odometry.hpp>
#include <localization_interfaces/msg/frontend_session.hpp>
#include <localization_interfaces/reset_transaction_cache.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>

#include "input.hpp"
#include "ros.hpp"
#include "zlio/odometry.hpp"
#include "zlio/prediction.hpp"

namespace zlio::ros {

/// 连接传感器输入、核心估计器和 ROS 输出。
class OdometryNode final : public rclcpp::Node {
public:
    /// 创建输入通道、估计对象和 ROS 接口。
    OdometryNode();
    /// 排空输入并结束估计线程。
    ~OdometryNode() override;

private:
    /// 保存尚未安装的估计器、预测器和输入队列。
    struct PreparedInput {
        std::unique_ptr<Odometry> odometry;
        std::unique_ptr<OutputPredictor> predictor;
        std::unique_ptr<InputQueue> queue;
    };
    /// 按当前配置构造完整替代资源，不改变运行会话。
    PreparedInput prepareInput() const;
    /// 从参数文件读取算法配置和 ROS 传输参数，并从 TF 取得雷达外参。
    void loadParameters();
    /// 阻塞等待 TF 中的 imu ← lidar 变换，返回雷达在 IMU 系中的位姿。
    Pose lookupBodyFromLidar();
    /// 查询 stamp 时刻车体在 IMU 系中的位姿（含云台关节）；不可用时返回 false。
    bool imuFromBase(const builtin_interfaces::msg::Time& stamp, Pose& pose);
    /// 输入雷达消息的 frame_id 与 frames.lidar 不一致时告警一次，不做变换。
    void checkLidarFrame(const std::string& frame_id);
    /// 建立后验、预测和会话发布接口。
    void setupPublishers();
    /// 为当前代次建立串行传感器回调。
    void setupSubscriptions();
    /// 建立当前会话的完成通知、传感器订阅和估计线程。
    void startInput();
    /// 串行关闭传感器订阅，按指定模式结束队列并汇合估计线程。
    void closeInput(InputQueue::CloseMode mode);
    /// 建立输入启用、停止和幂等重置服务。
    void setupServices();
    /// 关闭旧输入并在完整停止边界切换估计会话。
    void reset(const localization_interfaces::srv::ResetFrontend::Request& request,
               localization_interfaces::srv::ResetFrontend::Response& response);
    /// 接收 IMU，推进独立预测器并保存估计输入。
    void onImu(sensor_msgs::msg::Imu::ConstSharedPtr message);
    /// 解码 PointCloud2 并转交输入队列。
    void onCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
    /// 解码 Livox 原始回波并转交输入队列。
    void onLivox(livox_ros_driver2::msg::CustomMsg::ConstSharedPtr message);
    /// 关闭新准入并排空已接受输入。
    void requestStop();
    /// 锁存普通输入失败，取消待处理事件并禁止自动重试。
    void markInputFailed();
    /// 取消剩余输入并报告处理失败。
    void fail(const std::string& reason);
    /// 取消不连续输入并登记一次恢复请求，允许从回调或估计线程调用。
    void interruptInput(const std::string& reason);
    /// 在串行 ROS 回调内发布故障，并重新初始化或协调可恢复的输入中断。
    void recoverInput();
    /// 将完整后验交给独立预测器。
    void commit(const Estimate& estimate);
    /// 收集已就绪的观测结果并发布后验帧。
    void collect(bool flush = false);
    /// 顺序消费传感器事件并汇合停止状态。
    void work();
    /// 发布高频里程计、会话身份和坐标变换。
    void publish(const Estimate& estimate, const Vec3& angular_velocity);
    /// 发布当前前端进程身份和代次。
    void publishSession();

    Config config_;
    InputBufferLimits input_limits_;
    std::size_t maximum_queue_events_ = 0;
    std::unique_ptr<Odometry> odometry_;
    std::unique_ptr<OutputPredictor> predictor_;
    std::unique_ptr<InputQueue> queue_;
    std::unique_ptr<PointCloudDecoder> decoder_;
    OutputFrames frames_;
    PointCloudFormat cloud_format_;
    std::string lidar_frame_, base_frame_, input_type_, reset_mode_;
    double base_extrinsic_tolerance_ = 0;
    // 节点生命周期内唯一的 TF 缓存与监听器：启动时查 imu ← lidar，发布 world → base 时按时间查
    // imu ← base（云台转动会改变它）；last_imu_from_base_ 仅由串行 IMU 回调访问。
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    Pose last_imu_from_base_;
    bool has_imu_from_base_ = false;
    bool livox_timebase_ = false;
    bool publish_tf_ = false;
    bool coordinated_reset_ = false;
    // 雷达 frame_id 不一致告警只输出一次；仅由串行传感器回调访问。
    bool lidar_frame_warned_ = false;
    // 是否已提交首个后验；估计线程写入，控制回调在汇合旧线程后访问。
    bool has_posterior_ = false;
    // 串行控制回调维护并发布当前代次的协调重置需求。
    bool reset_required_ = false;
    // 输入回调或估计线程登记中断，由串行恢复回调消费。
    std::atomic<bool> pending_input_restart_{false};
    // 停止和普通失败禁止自动重试，新会话启动时解除。
    std::atomic<bool> allow_input_restart_{true};
    // 处理线程锁存普通失败，串行控制回调发布一次对应会话通知。
    std::atomic<bool> input_failed_{false};
    bool failure_reported_ = false;
    // 一次性输入启动门，仅由串行 ROS 回调访问，跨重置保留。
    bool armed_ = true;
    double output_resolution_ = 0;
    double imu_acceleration_scale_ = 0;
    std::uint64_t epoch_ = 0, sequence_ = 0;
    localization_interfaces::ResetTransactionCache reset_cache_;
    std::mutex prediction_mutex_;
    std::thread worker_;
    std::promise<void> finished_;
    std::shared_future<void> finished_future_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_, posterior_pub_;
    rclcpp::Publisher<localization_interfaces::msg::FrontendFrame>::SharedPtr frame_pub_;
    rclcpp::Publisher<localization_interfaces::msg::FrontendOdometry>::SharedPtr prediction_pub_;
    rclcpp::Publisher<localization_interfaces::msg::FrontendSession>::SharedPtr session_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_service_;
    rclcpp::Service<localization_interfaces::srv::ResetFrontend>::SharedPtr reset_service_;
    rclcpp::TimerBase::SharedPtr recovery_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
};

}  // namespace zlio::ros
