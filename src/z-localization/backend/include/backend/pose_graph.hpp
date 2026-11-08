/**
 * @file pose_graph.hpp
 * @brief 声明位姿图更新、观测线性化和状态查询接口。
 */
#pragma once

#include <gtsam/base/make_shared.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "backend/factors.hpp"
#include "backend/measurement.hpp"
#include "backend/types.hpp"

namespace backend {
/// 增量位姿图的数值求解失败异常。
class NumericalError final : public std::runtime_error {
public:
    /// 保存线性系统求解失败的具体原因。
    explicit NumericalError(const std::string& reason) : std::runtime_error(reason) {}
};

/// 管理增量位姿图的因子、状态估计和协方差查询。
class PoseGraph {
public:
    /// 构造增量位姿图并初始化固定窗口与噪声配置。
    explicit PoseGraph(const PoseGraphConfig& config = PoseGraphConfig());

    /// 清空当前图并应用新配置。
    void reset(const PoseGraphConfig& config);

    /// 使用配置的先验方差添加位姿先验，并登记节点初值和时间戳。
    void addPrior(int index, const Eigen::Matrix4d& pose,
                  double timestamp = std::numeric_limits<double>::quiet_NaN());
    /// 使用给定协方差添加位姿先验，并登记节点初值和时间戳。
    /// 协方差采用该位姿的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addPrior(int index, const Eigen::Matrix4d& pose, const PoseCovariance6d& covariance,
                  double timestamp = std::numeric_limits<double>::quiet_NaN());

    /// 添加前端相对位姿观测，以 initial_to_pose 登记目标节点初值。
    /// 协方差采用测量的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addOdometryMeasurement(int from_index, int to_index,
                                const Eigen::Matrix4d& measured_between,
                                const Eigen::Matrix4d& initial_to_pose,
                                const PoseCovariance6d& covariance,
                                double timestamp = std::numeric_limits<double>::quiet_NaN());

    /// 添加跨前端重置的相对位姿约束，并登记新代次的目标节点。
    /// 协方差采用测量的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addResetContinuityMeasurement(int from_index, int to_index,
                                       const Eigen::Matrix4d& measured_between,
                                       const Eigen::Matrix4d& initial_to_pose,
                                       const PoseCovariance6d& covariance,
                                       double timestamp = std::numeric_limits<double>::quiet_NaN());

    /// 基于前端重力方向添加横滚、俯仰约束，标准差单位为 rad。
    void addGravityAlignment(int index, const Eigen::Matrix4d& T_odom_sensor,
                             double standard_deviation_rad);

    /// 为已有节点添加完整协方差的绝对位姿观测。
    /// 协方差采用测量的局部右扰动坐标，顺序为 [平移, 旋转]。
    void addAbsolutePose(int index, const Eigen::Matrix4d& measured_pose,
                         const PoseCovariance6d& covariance);

    /// 在已提交图上计算绝对位姿观测的白化残差与雅可比。
    /// 平方根信息采用测量的局部右扰动坐标，列顺序为 [平移, 旋转]。
    InnovationTerm linearizeAbsolutePose(int index, const Eigen::Matrix4d& measured_pose,
                                         const PoseSqrtInformation& sqrt_information) const;
    /// 添加可降秩的绝对位姿因子，平方根信息采用测量局部右扰动的 [平移, 旋转] 列序。
    void addAbsolutePoseInformation(int index, const Eigen::Matrix4d& measured_pose,
                                    const PoseSqrtInformation& sqrt_information);
    /// 在已提交图上计算相对位姿观测的白化残差与雅可比。
    /// 平方根信息采用测量的局部右扰动坐标，列顺序为 [平移, 旋转]。
    InnovationTerm linearizeRelativePose(int from_index, int to_index,
                                         const Eigen::Matrix4d& measured_between,
                                         const PoseSqrtInformation& sqrt_information) const;
    /// 添加可降秩的相对位姿回环因子，并按配置应用鲁棒核。
    /// 平方根信息采用测量的局部右扰动坐标，列顺序为 [平移, 旋转]。
    void addLoopInformation(int from_index, int to_index, const Eigen::Matrix4d& measured_between,
                            const PoseSqrtInformation& sqrt_information);

    /// 基于更新前图协方差计算一批观测的联合及逐项创新检验结果。
    /// 调用时待提交因子与初值为空，各观测项来自同一图版本。
    InnovationBatchAssessment assessInnovationBatch(const std::vector<InnovationTerm>& terms) const;

    /// 提交待处理因子和初值，执行增量求解。
    /// 含全局因子时返回全部活动位姿，否则返回本批新增位姿。
    /// 线性系统求解失败时清空图并抛出 NumericalError。
    std::vector<std::pair<int, Eigen::Matrix4d>> update(bool global_factor_added);

    /// 判断指定关键帧是否仍有可查询的图估计。
    bool hasEstimate(int index) const;
    /// 查询指定关键帧的当前图位姿，状态不存在时返回空值。
    std::optional<Eigen::Matrix4d> estimate(int index) const;
    /// 返回位姿局部右扰动坐标下的边缘协方差，顺序为 [平移, 旋转]。
    std::optional<PoseCovariance6d> marginalCovariance(int index) const;

private:
    /// 将后端齐次位姿转换为 GTSAM Pose3。
    static gtsam::Pose3 toGtsam(const Eigen::Matrix4d& pose);
    /// 将 GTSAM Pose3 转换为后端齐次位姿。
    static Eigen::Matrix4d fromGtsam(const gtsam::Pose3& pose);
    /// 登记新位姿初值、时间戳和待提交的状态编号。
    void recordNewState(int index, const Eigen::Matrix4d& pose, double timestamp);
    /// 更新固定滞后窗口并边缘化过期轨迹位姿。
    void updateWindow();
    /// 按给定键顺序和变量切空间维数查询联合边缘协方差。
    std::optional<Eigen::MatrixXd> jointMarginalCovariance(
        const std::vector<gtsam::Key>& keys, const std::vector<Eigen::Index>& dimensions) const;
    /// 返回当前活动窗口内所有位姿的估计。
    std::vector<std::pair<int, Eigen::Matrix4d>> activeEstimates() const;

    /// 位姿图的噪声、鲁棒核与窗口配置。
    PoseGraphConfig config_;
    /// 当前增量求解器。
    std::unique_ptr<gtsam::ISAM2> isam_;
    /// 本批待提交的观测因子。
    gtsam::NonlinearFactorGraph pending_graph_;
    /// 本批新图变量的初值。
    gtsam::Values pending_values_;
    /// 固定滞后窗口中的轨迹节点键及其单调图时间，s。
    std::deque<std::pair<gtsam::Key, double>> pose_timestamps_;
    /// 本批新增的轨迹节点编号。
    std::vector<int> pending_new_indices_;
    /// 输入时间转换为单调图时间所使用的偏移，s。
    double timestamp_offset_ = 0.0;
    /// 最近一次登记的有效输入时间戳，s。
    double last_input_timestamp_ = std::numeric_limits<double>::quiet_NaN();
    /// 最近一次登记的单调图时间，s。
    double latest_graph_timestamp_ = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace backend
