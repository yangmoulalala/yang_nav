/**
 * @file config.hpp
 * @brief 定义后端 ROS 输入、输出和地图管理参数。
 */
#pragma once

#include <Eigen/Core>
#include <string>

#include "backend/config.hpp"
#include "backend/gprobe.hpp"
#include "backend/scan_context.hpp"

namespace backend::ros {

/// 前端消息、坐标系、同步器和维护周期参数。
/// 全部由参数文件提供；legacy 专用字段仅在 common.frontend=legacy 时读取，其余情况不使用。
/// ZLIO 与 UWB 输入话题为固定相对名，部署时通过 ROS remap 改名。
struct RosInputParameters {
    std::string frontend;
    std::string sensor_frame;
    std::string uwb_frame;
    std::string map_frame;
    std::string odom_frame;
    std::string body_frame;
    std::string odom_topic;
    std::string sync_odom_topic;
    std::string cloud_topic;
    bool cloud_in_local_frame = false;
    int subscription_queue = 1;
    int synchronizer_queue = 1;
    double synchronizer_max_interval = 0.0;
    double maintenance_period = 0.0;
    /// sensor <- body 外参按消息时间戳从 TF 查询；精确时刻不可用时允许使用的
    /// 最新变换与请求时刻的最大时间差，s（common.body_extrinsic_tolerance）。
    double body_extrinsic_tolerance = 0.0;
    /// UWB 天线相位中心在 body 系中的固定位置，m；按测量时刻外参换算到传感器系。
    Eigen::Vector3d antenna_position_body = Eigen::Vector3d::Zero();
};

/// 描述子类型、姿态整平及各实现的参数。
struct DescriptorParameters {
    std::string type = "scan_context";
    bool leveling = true;
    ScanContextDescriptorConfig scan_context;
    GProbeDescriptorConfig gprobe;
};

/// 先验图加载、运行轨迹保存和地图可视化频率参数。
struct MapStorageParameters {
    std::string prior_directory;
    double prior_voxel = 0.3;
    bool save_on_shutdown = false;
    bool save_global_pcd = true;
    std::string save_directory;
    double runtime_voxel = 0.3;
    double publish_period = 2.0;
};

/// 地图、轨迹、状态和配准点云的可视化开关。
struct VisualizationParameters {
    bool enabled = false;
    bool publish_prior_map = false;
    bool publish_runtime_map = false;
    bool publish_map_axes = false;
    bool publish_paths = false;
    bool publish_registration = false;
};

/// 前端重置客户端及后端公开服务的固定名称；不是参数，需要改名时使用 ROS remap（与话题一致）。
struct ServiceParameters {
    std::string frontend_reset = "/zlio/reset_internal";
    std::string save_map = "~/save_map";
    std::string relocalize = "~/relocalize";
    std::string reset = "/localization_reset";
};

/// 世界坐标系实时点云的发布参数。
struct WorldCloudOutputParameters {
    bool enabled = false;
    double publish_period = 0.0;
    bool publish_initializing = false;
    bool publish_untrusted = false;
};

/// 汇集后端核心参数与 ROS 接口配置。
struct NodeConfig {
    Config runtime;
    RosInputParameters input;
    DescriptorParameters descriptor;
    MapStorageParameters maps;
    VisualizationParameters visualization;
    ServiceParameters services;
    WorldCloudOutputParameters world_cloud;
};

/// 校验 UWB 输入坐标系名称与固定变换、在线空间标定开关是否一致。
inline bool uwbFrameContractValid(const UwbConfig& uwb, const std::string& source_frame,
                                  const std::string& map_frame) {
    if (!uwb.enabled) {
        return true;
    }
    if (source_frame.empty()) {
        return false;
    }
    if (source_frame != map_frame) {
        return true;
    }
    return !uwb.estimate_zero_pose_online &&
           uwb.T_map_uwb_fixed.isApprox(Eigen::Matrix4d::Identity(), 1.0e-12);
}

}  // namespace backend::ros
