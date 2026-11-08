/**
 * @file factors.hpp
 * @brief 定义图观测输入与创新检验结果。
 */
#pragma once
#include <gtsam/inference/Key.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "backend/config.hpp"
#include "backend/measurement.hpp"

namespace backend {
/// 单项或联合观测的创新检验结果。
struct InnovationAssessment {
    /// 检验结果是否有效。
    bool valid = false;
    /// 卡方检验的自由度。
    int degrees_of_freedom = 0;
    /// 归一化创新平方（NIS）。
    double nis = std::numeric_limits<double>::infinity();
    /// 卡方分布的上尾概率。
    double upper_tail_probability = 0.0;
    /// 检验结果无效时的原因。
    std::string invalid_reason;
};

/// 外部观测在已提交图预测点处的白化残差与雅可比。
struct InnovationTerm {
    /// 观测项是否已成功线性化与白化。
    bool valid = false;
    /// 观测涉及的图变量键，与雅可比块顺序一致。
    std::vector<gtsam::Key> keys;
    /// 白化残差，对应的测量噪声协方差为单位阵。
    Eigen::VectorXd whitened_residual;
    /// 各图变量的白化雅可比；列数为其切空间维数：Pose3 为 6、Pose2 为 3、标量为 1。
    std::vector<Eigen::MatrixXd> whitened_jacobians;
    /// 观测项无效时的原因。
    std::string invalid_reason;
};

/// 同一批观测基于更新前图协方差得到的创新检验结果。
struct InnovationBatchAssessment {
    /// 全部观测的联合检验结果。
    InnovationAssessment joint;
    /// 各观测的逐项检验结果，顺序与输入观测项一致。
    std::vector<InnovationAssessment> individual;
};

/// 将逐分量白化残差的 RMS 阈值换算为 GTSAM 块鲁棒核的残差范数阈值。
inline double robustBlockKernelSize(const double per_component_rms, const std::size_t dimension) {
    return per_component_rms * std::sqrt(static_cast<double>(std::max<std::size_t>(1, dimension)));
}

}  // namespace backend
