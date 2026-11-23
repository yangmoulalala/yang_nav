/**
 * @file parameters.hpp
 * @brief 声明里程计节点从 ROS 参数文件读取的算法与传输配置。
 */
#pragma once

#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "ros.hpp"
#include "zlio/config.hpp"
#include "zlio/input.hpp"

namespace zlio::ros {

/// 汇集里程计节点的全部参数；各字段均来自参数文件，不使用代码默认值。
struct NodeParameters {
    /// 算法配置；外参 body_from_lidar 尚未填写，由 TF 查询后写入并规范化。
    Config config;
    /// IMU 历史与点云缓存容量。
    InputBufferLimits input_limits;
    /// ROS 待处理队列的事件数上限。
    std::size_t maximum_queue_events = 0;
    /// 里程计父坐标系与 IMU 子坐标系。
    OutputFrames frames;
    /// LiDAR 坐标系，用于查询 imu ← lidar 外参。
    std::string lidar_frame;
    /// TF 输出的子坐标系（车体）；与 frames.imu 相同时直接发布 world → imu。
    std::string base_frame;
    /// 按时间查询 imu ← base 失败时，最新变换允许的最大时间差，s。
    double base_extrinsic_tolerance = 0;
    /// PointCloud2 逐点时间格式。
    PointCloudFormat cloud_format;
    /// LiDAR 消息类型：pointcloud2 或 livox。
    std::string input_type;
    /// 重置模式：warm 或 cold。
    std::string reset_mode;
    bool livox_timebase = false;
    bool publish_tf = false;
    bool wait_for_start = false;
    bool coordinated_reset = false;
    double output_resolution = 0;
    double imu_acceleration_scale = 0;
};

/// 读取并校验里程计节点的全部必填参数，拒绝未知参数。
NodeParameters loadNodeParameters(rclcpp::Node& node);

}  // namespace zlio::ros
