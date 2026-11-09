/**
 * @file graph_estimates.cpp
 * @brief 查询轨迹位姿及其边缘协方差。
 */

#include <array>
#include <cmath>
#include <limits>

#include "backend/pose_graph.hpp"

namespace backend {
namespace {

/// 将 GTSAM 的旋转优先切空间协方差转换为后端的平移优先顺序。
PoseCovariance6d gtsamToRosOrder(const gtsam::Matrix& covariance) {
    PoseCovariance6d out = PoseCovariance6d::Zero();
    if (covariance.rows() != 6 || covariance.cols() != 6) {
        return out;
    }
    // GTSAM 的排列顺序为 [旋转, 平移]，后端及 ROS 为 [平移, 旋转]。
    out.block<3, 3>(0, 0) = covariance.block<3, 3>(3, 3);
    out.block<3, 3>(0, 3) = covariance.block<3, 3>(3, 0);
    out.block<3, 3>(3, 0) = covariance.block<3, 3>(0, 3);
    out.block<3, 3>(3, 3) = covariance.block<3, 3>(0, 0);
    return 0.5 * (out + out.transpose());
}

}  // namespace

/// 返回当前求解器内全部活动轨迹位姿，不包含共享标定变量。
std::vector<std::pair<int, Eigen::Matrix4d>> PoseGraph::activeEstimates() const {
    const gtsam::Values values = isam_->calculateEstimate();
    std::vector<std::pair<int, Eigen::Matrix4d>> poses;
    poses.reserve(values.size());
    for (const gtsam::Key key : values.keys()) {
        // 轨迹编号使用非负 int，带字符前缀的共享变量属于独立键空间。
        if (key > static_cast<gtsam::Key>(std::numeric_limits<int>::max())) {
            continue;
        }
        poses.emplace_back(static_cast<int>(key), fromGtsam(values.at<gtsam::Pose3>(key)));
    }
    return poses;
}

/// 判断指定轨迹节点是否仍有活动估计。
bool PoseGraph::hasEstimate(const int index) const {
    return isam_->valueExists(index);
}

/// 查询指定轨迹节点位姿；节点不存在时返回空值。
std::optional<Eigen::Matrix4d> PoseGraph::estimate(const int index) const {
    if (!isam_->valueExists(index)) {
        return std::nullopt;
    }
    return fromGtsam(isam_->calculateEstimate<gtsam::Pose3>(index));
}

/// 查询轨迹位姿的局部右扰动边缘协方差，并转换为后端 [平移, 旋转] 顺序。
std::optional<PoseCovariance6d> PoseGraph::marginalCovariance(const int index) const {
    if (!isam_->valueExists(index)) {
        return std::nullopt;
    }
    try {
        return gtsamToRosOrder(isam_->marginalCovariance(index));
    } catch (const std::exception&) {
        return std::nullopt;
    }
}

}  // namespace backend
