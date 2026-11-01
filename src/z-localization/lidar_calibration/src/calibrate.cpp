/**
 * @file calibrate.cpp
 * @brief 副雷达到主雷达的外参标定（节点 calibrate）。
 *
 * 流程沿用 navi_minco_bit/dual_lidar_calibration 的核心：体素降采样 -> PCL GICP ->
 * 重叠率/RMSE 质量门 -> 平移中值剔除离群 -> 加权平移 + 四元数平均。
 * 与原工具的区别：输入是任意 PointCloud2（不绑定 Livox），只用静止帧（主雷达 IMU 角速度与
 * 里程计线速度都低于阈值），因此不需要逐点去畸变；初值和输出都走 tf2（URDF），结果直接给出 URDF 关节 origin。
 *
 * 单个位置的点云往往只约束部分方向（地面 + 一面墙），GICP 会沿未约束方向滑走，
 * 而且同一位置的多帧结果彼此一致，离散度检查拦不住。因此每帧还要通过：
 *   - 采样位姿：与已接受帧的里程计位姿相距足够远（多位置、多朝向）；
 *   - 几何退化：重叠区目标法向信息矩阵的最小特征值（日志打印每帧平移与最弱方向）；
 *   - 点到面 RMSE。
 *
 * 参数只来自 ROS 参数文件：全部必填、无默认值，缺失、类型错误或未知参数都会使启动失败。
 * 话题为固定相对名（primary/points、secondary/points、imu、odometry），由 launch 重映射。
 *
 *   ros2 launch lidar_calibration calibrate.launch.py use_sim_time:=true
 *
 * 采集方法：把车停在不同位置/朝向（间隔 >= min_pose_translation 或 >= min_pose_rotation），
 * 每处静止 2 s 以上，直到累计满 frames 帧。
 */
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct FrameResult {
    Eigen::Isometry3d transform;
    double rmse;
    double overlap;
    double support;
};

/// 里程计给出的平面采样位姿。
struct PlanarPose {
    double x = 0, y = 0, yaw = 0;
};

/// 绕各轴固定轴 rpy（URDF 约定）转换为旋转矩阵。
Eigen::Matrix3d fromRpy(double roll, double pitch, double yaw) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

/// 返回 URDF 约定的 roll、pitch、yaw。
Eigen::Vector3d toRpy(const Eigen::Matrix3d& R) {
    const Eigen::Vector3d ypr = R.eulerAngles(2, 1, 0);
    return {ypr.z(), ypr.y(), ypr.x()};
}

double angleBetween(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
    return Eigen::AngleAxisd(a.transpose() * b).angle();
}

/// 读取参数文件提供的必填参数，记录已消费名称以便拒绝未知参数（同 frontend/apps/parameter_reader.hpp）。
class ParameterReader {
public:
    explicit ParameterReader(rclcpp::Node& node) : node_(node) {}

    /// 浮点参数，兼容整数 YAML 写法。
    double real(const std::string& name) {
        const auto p = require(name);
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            return static_cast<double>(p.as_int());
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE)
            throw typeError(p, "double");
        return p.as_double();
    }

    /// 正浮点参数。
    double positive(const std::string& name) {
        const double value = real(name);
        if (!(value > 0))
            throw std::invalid_argument("parameter '" + name + "' must be positive");
        return value;
    }

    /// 不小于 minimum 的整数参数。
    int integer(const std::string& name, std::int64_t minimum) {
        const auto p = require(name);
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER)
            throw typeError(p, "integer");
        if (p.as_int() < minimum)
            throw std::invalid_argument("parameter '" + name + "' must be >= " + std::to_string(minimum));
        return static_cast<int>(p.as_int());
    }

    bool boolean(const std::string& name) {
        const auto p = require(name);
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_BOOL)
            throw typeError(p, "bool");
        return p.as_bool();
    }

    /// 非空字符串参数。
    std::string text(const std::string& name) {
        const auto p = require(name);
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING)
            throw typeError(p, "string");
        if (p.as_string().empty())
            throw std::invalid_argument("parameter '" + name + "' must not be empty");
        return p.as_string();
    }

    /// 固定长度浮点数组，兼容整数数组写法。
    std::vector<double> reals(const std::string& name, std::size_t length) {
        const auto p = require(name);
        std::vector<double> values;
        if (p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
            const auto& integers = p.as_integer_array();
            values.assign(integers.begin(), integers.end());
        } else if (p.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
            values = p.as_double_array();
        } else {
            throw typeError(p, "double array");
        }
        if (values.size() != length)
            throw std::invalid_argument("parameter '" + name + "' must have exactly " +
                                        std::to_string(length) + " elements");
        return values;
    }

    /// 拒绝参数文件中未被读取的参数，框架参数除外。
    void rejectUnknown() const {
        std::vector<std::string> unknown;
        for (const auto& name : node_.list_parameters({}, 0).names)
            if (consumed_.count(name) == 0 && name != "use_sim_time" &&
                name != "start_type_description_service" && name.rfind("qos_overrides.", 0) != 0)
                unknown.push_back(name);
        if (unknown.empty())
            return;
        std::ostringstream message;
        message << "unknown parameters for node " << node_.get_fully_qualified_name()
                << " (check spelling against lidar_calibration/README.md): ";
        for (std::size_t i = 0; i < unknown.size(); ++i)
            message << (i == 0 ? "" : ", ") << unknown[i];
        throw std::invalid_argument(message.str());
    }

private:
    rclcpp::Parameter require(const std::string& name) {
        consumed_.insert(name);
        if (!node_.has_parameter(name) ||
            node_.get_parameter(name).get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET)
            throw std::invalid_argument("missing required parameter '" + name + "' for node " +
                                        node_.get_fully_qualified_name() +
                                        "; provide it in the ROS params file");
        return node_.get_parameter(name);
    }

    static std::invalid_argument typeError(const rclcpp::Parameter& p, const char* expected) {
        return std::invalid_argument("parameter '" + p.get_name() + "' must be of type " + expected +
                                     ", got " + p.get_type_name());
    }

    rclcpp::Node& node_;
    std::unordered_set<std::string> consumed_;
};

}  // namespace

class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode()
        : Node("calibrate", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
          tf_buffer_(get_clock()),
          tf_listener_(tf_buffer_) {
        ParameterReader read(*this);
        primary_frame_ = read.text("primary_frame");
        secondary_frame_ = read.text("secondary_frame");
        joint_parent_ = read.text("secondary_parent_frame");
        // true：初值取 tf2（URDF）中 primary_frame <- secondary_frame；false：取 initial_xyz_rpy。
        initial_from_tf_ = read.boolean("initial_guess_from_tf");
        // 始终必须是 6 个数 [x, y, z, roll, pitch, yaw]（m, rad，URDF 约定），initial_guess_from_tf=true 时不使用。
        const auto xyz_rpy = read.reals("initial_xyz_rpy", 6);
        initial_ = Eigen::Isometry3d::Identity();
        initial_.linear() = fromRpy(xyz_rpy[3], xyz_rpy[4], xyz_rpy[5]);
        initial_.translation() = Eigen::Vector3d(xyz_rpy[0], xyz_rpy[1], xyz_rpy[2]);
        frames_needed_ = read.integer("frames", 3);
        period_ = read.real("min_frame_period");
        sync_tolerance_ = read.positive("sync_tolerance");
        stationary_gyro_ = read.positive("stationary_gyro_threshold");
        // 匀速直线时角速度接近 0，仅看陀螺会把运动误判为静止；再用里程计线速度判定。
        stationary_speed_ = read.positive("stationary_speed_threshold");
        min_range_ = read.real("min_range");
        voxel_ = read.positive("voxel_size");
        max_correspondence_ = read.positive("max_correspondence_distance");
        min_overlap_ = read.real("min_overlap_ratio");
        min_inliers_ = read.integer("min_inliers", 1);
        max_rmse_ = read.positive("max_rmse");
        min_support_ = read.real("min_normal_support");
        normal_k_ = read.integer("normal_k", 3);
        // true：每帧须与已接受帧的里程计位姿拉开距离或转角；false：不检查（不推荐）。
        require_distinct_poses_ = read.boolean("require_distinct_poses");
        min_pose_translation_ = read.real("min_pose_translation");
        min_pose_rotation_ = read.real("min_pose_rotation");
        // 运行时选择，由 launch 给出，不写入参数文件。
        output_file_ = read.text("output_file");
        read.rejectUnknown();

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
                const auto& w = m->angular_velocity;
                if (std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z) > stationary_gyro_)
                    last_motion_ = rclcpp::Time(m->header.stamp);
            });
        // 里程计始终订阅：线速度用于静止判定，位姿用于多样性检查（require_distinct_poses）。
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "odometry", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                    const auto& v = m->twist.twist.linear;
                    if (std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z) > stationary_speed_)
                        last_motion_ = rclcpp::Time(m->header.stamp);
                    has_speed_ = true;
                    const auto& p = m->pose.pose;
                    const auto& q = p.orientation;
                    pose_ = {p.position.x, p.position.y,
                             std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
                    has_pose_ = true;
                });

        // 时间配对交给 message_filters 的 ApproximateTime 策略。
        const auto qos = rclcpp::QoS(5).reliable();
        primary_sub_.subscribe(this, "primary/points", qos.get_rmw_qos_profile());
        secondary_sub_.subscribe(this, "secondary/points", qos.get_rmw_qos_profile());
        sync_ = std::make_shared<Sync>(Policy(10), primary_sub_, secondary_sub_);
        sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_));
        sync_->registerCallback(&CalibrationNode::onPair, this);
        RCLCPP_INFO(get_logger(), "collecting %d stationary frame pairs (%s <- %s)%s, initial guess from %s",
                    frames_needed_, primary_frame_.c_str(), secondary_frame_.c_str(),
                    require_distinct_poses_ ? " from distinct odometry poses" : " (pose diversity NOT checked)",
                    initial_from_tf_ ? "TF" : "initial_xyz_rpy");
    }

private:
    /// 读取点云并按 tf2 变换到目标坐标系，剔除近距点后体素降采样。
    Cloud::Ptr prepare(const PointCloud2& message, const std::string& frame) {
        Cloud raw;
        pcl::fromROSMsg(message, raw);
        const Eigen::Isometry3d T = tf2::transformToEigen(
            tf_buffer_.lookupTransform(frame, message.header.frame_id, tf2::TimePointZero));
        Cloud::Ptr kept(new Cloud);
        for (const auto& p : raw)
            if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
                p.getVector3fMap().norm() >= min_range_)
                kept->push_back(p);
        pcl::transformPointCloud(*kept, *kept, T.matrix().cast<float>());
        Cloud::Ptr filtered(new Cloud);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(voxel_, voxel_, voxel_);
        voxel.setInputCloud(kept);
        voxel.filter(*filtered);
        return filtered;
    }

    /// 外参初值：initial_guess_from_tf 为 true 时取 tf2（URDF），否则取 initial_xyz_rpy。
    Eigen::Isometry3d initialGuess() {
        if (!initial_from_tf_)
            return initial_;
        return tf2::transformToEigen(
            tf_buffer_.lookupTransform(primary_frame_, secondary_frame_, tf2::TimePointZero));
    }

    /// 当前位姿与所有已接受帧的位姿都拉开距离或转角时返回 true。
    bool newPose(const PlanarPose& p) const {
        for (const auto& q : accepted_poses_) {
            const double yaw = std::abs(std::remainder(p.yaw - q.yaw, 2 * M_PI));
            if (std::hypot(p.x - q.x, p.y - q.y) < min_pose_translation_ && yaw < min_pose_rotation_)
                return false;
        }
        return true;
    }

    void onPair(const PointCloud2::ConstSharedPtr& primary, const PointCloud2::ConstSharedPtr& secondary) {
        if (done_)
            return;
        const rclcpp::Time stamp(primary->header.stamp);
        if ((stamp - last_motion_).seconds() < 0.5 || (stamp - last_frame_).seconds() < period_)
            return;
        if (!has_speed_ || (require_distinct_poses_ && !has_pose_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for odometry");
            return;
        }
        if (require_distinct_poses_ && !newPose(pose_))
            return;
        try {
            const auto target = prepare(*primary, primary_frame_);
            const auto source = prepare(*secondary, secondary_frame_);
            const auto initial = initialGuess();
            last_frame_ = stamp;
            const PlanarPose pose = pose_;
            if (register_(target, source, initial))
                accepted_poses_.push_back(pose);
        } catch (const tf2::TransformException& error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for TF: %s", error.what());
        }
    }

    /// 单帧 GICP 与质量门，结果满足帧数后聚合输出；返回本帧是否被接受。
    bool register_(const Cloud::Ptr& target, const Cloud::Ptr& source, const Eigen::Isometry3d& initial) {
        if (target->size() < static_cast<std::size_t>(min_inliers_) ||
            source->size() < static_cast<std::size_t>(min_inliers_))
            return reject("too_few_points");
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        gicp.setInputTarget(target);
        gicp.setInputSource(source);
        gicp.setMaxCorrespondenceDistance(max_correspondence_);
        gicp.setMaximumIterations(64);
        gicp.setTransformationEpsilon(1e-6);
        Cloud aligned;
        gicp.align(aligned, initial.matrix().cast<float>());
        if (!gicp.hasConverged())
            return reject("not_converged");

        // 目标法向：点到面残差与几何退化检查共用。
        pcl::PointCloud<pcl::Normal> normals;
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> estimation;
        estimation.setInputCloud(target);
        estimation.setKSearch(normal_k_);
        estimation.compute(normals);

        pcl::KdTreeFLANN<pcl::PointXYZ> tree;
        tree.setInputCloud(target);
        std::vector<int> index(1);
        std::vector<float> distance(1);
        std::size_t inliers = 0;
        double squared = 0;
        Eigen::Matrix3d information = Eigen::Matrix3d::Zero();
        for (const auto& p : aligned) {
            if (tree.nearestKSearch(p, 1, index, distance) <= 0 ||
                distance[0] > max_correspondence_ * max_correspondence_)
                continue;
            const auto& normal = normals[index[0]];
            if (!std::isfinite(normal.normal_x))
                continue;
            const Eigen::Vector3d n(normal.normal_x, normal.normal_y, normal.normal_z);
            const double residual =
                n.dot((p.getVector3fMap() - (*target)[index[0]].getVector3fMap()).cast<double>());
            ++inliers;
            squared += residual * residual;
            information += n * n.transpose();
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(
            inliers ? Eigen::Matrix3d(information / inliers) : Eigen::Matrix3d::Zero());
        const double support = inliers ? eigen.eigenvalues()[0] : 0.0;
        // 最弱约束方向（主雷达系）：平移沿该方向最不可靠，用于诊断。
        const Eigen::Vector3d weak = eigen.eigenvectors().col(0);
        FrameResult frame{Eigen::Isometry3d(gicp.getFinalTransformation().cast<double>()),
                          inliers ? std::sqrt(squared / inliers) : INFINITY,
                          static_cast<double>(inliers) / aligned.size(), support};
        std::ostringstream detail;
        const Eigen::Vector3d t = frame.transform.translation();
        detail << std::fixed << std::setprecision(3) << " (rmse=" << frame.rmse << " overlap=" << frame.overlap
               << " support=" << frame.support << " t=[" << t.x() << ", " << t.y() << ", " << t.z()
               << "] weak=[" << weak.x() << ", " << weak.y() << ", " << weak.z() << "])";
        if (inliers < static_cast<std::size_t>(min_inliers_))
            return reject("insufficient_inliers" + detail.str());
        if (frame.overlap < min_overlap_)
            return reject("insufficient_overlap" + detail.str());
        if (frame.support < min_support_)
            return reject("degenerate_geometry" + detail.str());
        if (frame.rmse > max_rmse_)
            return reject("rmse_too_large" + detail.str());
        results_.push_back(frame);
        RCLCPP_INFO(get_logger(), "frame %zu/%d accepted%s", results_.size(), frames_needed_,
                    detail.str().c_str());
        if (results_.size() >= static_cast<std::size_t>(frames_needed_))
            finish();
        return true;
    }

    bool reject(const std::string& reason) {
        RCLCPP_INFO(get_logger(), "frame rejected: %s", reason.c_str());
        return false;
    }

    /// 平移逐轴中值剔除离群，再按 overlap/rmse 加权平均平移与四元数。
    void finish() {
        Eigen::Vector3d median;
        for (int axis = 0; axis < 3; ++axis) {
            std::vector<double> v;
            for (const auto& r : results_)
                v.push_back(r.transform.translation()[axis]);
            std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
            median[axis] = v[v.size() / 2];
        }
        const auto& reference = *std::min_element(
            results_.begin(), results_.end(), [&](const FrameResult& a, const FrameResult& b) {
                return (a.transform.translation() - median).norm() <
                       (b.transform.translation() - median).norm();
            });
        const Eigen::Quaterniond q_ref(reference.transform.linear());
        Eigen::Vector3d translation = Eigen::Vector3d::Zero();
        Eigen::Vector4d q_sum = Eigen::Vector4d::Zero();
        double weights = 0;
        std::vector<const FrameResult*> used;
        for (const auto& r : results_) {
            if ((r.transform.translation() - median).norm() > 0.1 ||
                angleBetween(r.transform.linear(), reference.transform.linear()) > 0.05)
                continue;
            const double w = std::max(1e-6, r.overlap) / std::max(1e-3, r.rmse);
            Eigen::Quaterniond q(r.transform.linear());
            if (q.coeffs().dot(q_ref.coeffs()) < 0)
                q.coeffs() *= -1;
            translation += w * r.transform.translation();
            q_sum += w * q.coeffs();
            weights += w;
            used.push_back(&r);
        }
        done_ = true;
        if (used.size() < 3) {
            RCLCPP_ERROR(get_logger(), "only %zu consistent frames; calibration failed", used.size());
            rclcpp::shutdown();
            return;
        }
        Eigen::Quaterniond q;
        q.coeffs() = q_sum.normalized();
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = q.toRotationMatrix();
        T.translation() = translation / weights;
        double t_var = 0, r_var = 0, support = INFINITY;
        for (const auto* r : used) {
            t_var += (r->transform.translation() - T.translation()).squaredNorm();
            r_var += std::pow(angleBetween(r->transform.linear(), T.linear()), 2);
            support = std::min(support, r->support);
        }
        write(T, used.size(), std::sqrt(t_var / used.size()), std::sqrt(r_var / used.size()), support);
        rclcpp::shutdown();
    }

    void write(const Eigen::Isometry3d& T, std::size_t used, double t_std, double r_std, double support) {
        // URDF 关节 origin：parent -> secondary = parent -> primary (tf2) * 标定结果。
        const Eigen::Isometry3d joint =
            tf2::transformToEigen(
                tf_buffer_.lookupTransform(joint_parent_, primary_frame_, tf2::TimePointZero)) *
            T;
        const auto line = [](const Eigen::Vector3d& v) {
            std::ostringstream s;
            s << std::fixed << std::setprecision(6) << '[' << v.x() << ", " << v.y() << ", " << v.z()
              << ']';
            return s.str();
        };
        std::ofstream out(output_file_);
        out << "# " << secondary_frame_ << " -> " << primary_frame_ << " (p_primary = T * p_secondary)\n"
            << "primary_frame: " << primary_frame_ << "\nsecondary_frame: " << secondary_frame_ << '\n'
            << "xyz: " << line(T.translation()) << "\nrpy: " << line(toRpy(T.linear())) << '\n'
            << "used_frames: " << used << "\ntranslation_stddev: " << t_std
            << "\nrotation_stddev_rad: " << r_std << "\nmin_normal_support: " << support << '\n'
            << "# URDF <origin> of the joint " << joint_parent_ << " -> " << secondary_frame_ << '\n'
            << "urdf_joint:\n  parent: " << joint_parent_ << "\n  xyz: " << line(joint.translation())
            << "\n  rpy: " << line(toRpy(joint.linear())) << '\n';
        if (!out)
            RCLCPP_ERROR(get_logger(), "failed to write %s", output_file_.c_str());
        RCLCPP_INFO(get_logger(), "calibrated from %zu frames: xyz=%s rpy=%s (std %.4f m, %.4f rad) -> %s",
                    used, line(T.translation()).c_str(), line(toRpy(T.linear())).c_str(), t_std,
                    r_std, output_file_.c_str());
    }

    using Policy = message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>;
    using Sync = message_filters::Synchronizer<Policy>;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    message_filters::Subscriber<PointCloud2> primary_sub_, secondary_sub_;
    std::shared_ptr<Sync> sync_;
    rclcpp::Time last_motion_{0, 0, RCL_ROS_TIME}, last_frame_{0, 0, RCL_ROS_TIME};
    PlanarPose pose_;
    bool has_pose_ = false;
    // 收到过里程计（线速度静止判定可用）之前不采帧。
    bool has_speed_ = false;
    std::vector<PlanarPose> accepted_poses_;
    std::vector<FrameResult> results_;
    bool done_ = false;
    std::string primary_frame_, secondary_frame_, joint_parent_, output_file_;
    bool initial_from_tf_ = true, require_distinct_poses_ = true;
    Eigen::Isometry3d initial_ = Eigen::Isometry3d::Identity();
    int frames_needed_ = 0, min_inliers_ = 0, normal_k_ = 0;
    double period_ = 0, sync_tolerance_ = 0, stationary_gyro_ = 0, stationary_speed_ = 0,
           min_range_ = 0, voxel_ = 0,
           max_correspondence_ = 0, min_overlap_ = 0, max_rmse_ = 0, min_support_ = 0,
           min_pose_translation_ = 0, min_pose_rotation_ = 0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        rclcpp::spin(std::make_shared<CalibrationNode>());
    } catch (const std::exception& error) {
        std::cerr << "calibrate failed: " << error.what() << '\n';
        status = 1;
    }
    if (rclcpp::ok())
        rclcpp::shutdown();
    return status;
}
