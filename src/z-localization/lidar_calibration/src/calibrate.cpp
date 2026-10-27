/**
 * @file calibrate.cpp
 * @brief 副雷达到主雷达的外参标定。
 *
 * 流程沿用 navi_minco_bit/dual_lidar_calibration 的核心：体素降采样 -> PCL GICP ->
 * 重叠率/RMSE 质量门 -> 平移中值剔除离群 -> 加权平移 + 四元数平均。
 * 与原工具的区别：输入是任意 PointCloud2（不绑定 Livox），只用主雷达 IMU 判定静止的帧，
 * 因此不需要逐点去畸变；初值和输出都走 tf2（URDF），结果直接给出 URDF 关节 origin。
 *
 * 单个位置的点云往往只约束部分方向（地面 + 一面墙），GICP 会沿未约束方向滑走，
 * 而且同一位置的多帧结果彼此一致，离散度检查拦不住。因此每帧还要通过：
 *   - 采样位姿：与已接受帧的里程计位姿相距足够远（多位置、多朝向）；
 *   - 几何退化：重叠区目标法向信息矩阵的最小特征值；
 *   - 点到面 RMSE。
 *
 *   ros2 run lidar_calibration calibrate --ros-args -p use_sim_time:=true
 *
 * 采集方法：把车停在不同位置/朝向（间隔 >= 0.3 m 或 >= 20 度），每处静止 2 s 以上，
 * 直到累计满 frames 帧。
 */
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
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

}  // namespace

class CalibrationNode : public rclcpp::Node {
public:
    CalibrationNode()
        : Node("lidar_calibration"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
        primary_frame_ = declare_parameter<std::string>("primary_frame", "mid360_link");
        secondary_frame_ = declare_parameter<std::string>("secondary_frame", "l2_link");
        joint_parent_ = declare_parameter<std::string>("secondary_parent_frame", "yaw_link");
        initial_ = declare_parameter<std::vector<double>>("initial_xyz_rpy", {0.0});
        frames_needed_ = declare_parameter<int>("frames", 6);
        period_ = declare_parameter<double>("min_frame_period", 0.5);
        sync_tolerance_ = declare_parameter<double>("sync_tolerance", 0.05);
        stationary_gyro_ = declare_parameter<double>("stationary_gyro_threshold", 0.03);
        min_range_ = declare_parameter<double>("min_range", 0.3);
        voxel_ = declare_parameter<double>("voxel_size", 0.1);
        max_correspondence_ = declare_parameter<double>("max_correspondence_distance", 0.5);
        min_overlap_ = declare_parameter<double>("min_overlap_ratio", 0.3);
        min_inliers_ = declare_parameter<int>("min_inliers", 300);
        // 点到面残差，m。
        max_rmse_ = declare_parameter<double>("max_rmse", 0.05);
        // 重叠区法向信息矩阵（按点数归一化）最小特征值下限；平面各占 1/3 时约为 0.33。
        min_support_ = declare_parameter<double>("min_normal_support", 0.05);
        normal_k_ = declare_parameter<int>("normal_k", 10);
        min_pose_translation_ = declare_parameter<double>("min_pose_translation", 0.3);
        min_pose_rotation_ = declare_parameter<double>("min_pose_rotation", 0.35);
        output_file_ = declare_parameter<std::string>("output_file", "/tmp/lidar_extrinsic.yaml");

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            declare_parameter<std::string>("primary_imu_topic", "/lidar_mid360/imu"),
            rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
                const auto& w = m->angular_velocity;
                if (std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z) > stationary_gyro_)
                    last_motion_ = rclcpp::Time(m->header.stamp);
            });
        // 采样位姿来源；为空时不检查位姿多样性（不推荐）。
        const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
        if (!odom_topic.empty())
            odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
                    const auto& p = m->pose.pose;
                    const auto& q = p.orientation;
                    pose_ = {p.position.x, p.position.y,
                             std::atan2(2 * (q.w * q.z + q.x * q.y),
                                        1 - 2 * (q.y * q.y + q.z * q.z))};
                    has_pose_ = true;
                });

        // 时间配对交给 message_filters 的 ApproximateTime 策略。
        const auto qos = rclcpp::QoS(5).reliable();
        primary_sub_.subscribe(this, declare_parameter<std::string>("primary_topic", "/lidar_mid360/points"),
                               qos.get_rmw_qos_profile());
        secondary_sub_.subscribe(this, declare_parameter<std::string>("secondary_topic", "/lidar_l2/points"),
                                 qos.get_rmw_qos_profile());
        sync_ = std::make_shared<Sync>(Policy(10), primary_sub_, secondary_sub_);
        sync_->getPolicy()->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_tolerance_));
        sync_->registerCallback(&CalibrationNode::onPair, this);
        RCLCPP_INFO(get_logger(), "collecting %d stationary frame pairs (%s <- %s) from distinct poses",
                    frames_needed_, primary_frame_.c_str(), secondary_frame_.c_str());
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

    /// 外参初值：参数给出 6 个数时使用参数，否则取 tf2（URDF）。
    Eigen::Isometry3d initialGuess() {
        if (initial_.size() == 6) {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.linear() = fromRpy(initial_[3], initial_[4], initial_[5]);
            T.translation() = Eigen::Vector3d(initial_[0], initial_[1], initial_[2]);
            return T;
        }
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
        if (odom_sub_ && !has_pose_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for odometry");
            return;
        }
        if (odom_sub_ && !newPose(pose_))
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
        const double support =
            inliers ? Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(information / inliers).eigenvalues()[0]
                    : 0.0;
        FrameResult frame{Eigen::Isometry3d(gicp.getFinalTransformation().cast<double>()),
                          inliers ? std::sqrt(squared / inliers) : INFINITY,
                          static_cast<double>(inliers) / aligned.size(), support};
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(3) << " (rmse=" << frame.rmse << " overlap=" << frame.overlap
               << " support=" << frame.support << ')';
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
        // URDF 关节 origin：parent(yaw_link) -> secondary = parent -> primary (tf2) * 标定结果。
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
    std::vector<PlanarPose> accepted_poses_;
    std::vector<FrameResult> results_;
    bool done_ = false;
    std::string primary_frame_, secondary_frame_, joint_parent_, output_file_;
    std::vector<double> initial_;
    int frames_needed_, min_inliers_, normal_k_;
    double period_, sync_tolerance_, stationary_gyro_, min_range_, voxel_, max_correspondence_,
        min_overlap_, max_rmse_, min_support_, min_pose_translation_, min_pose_rotation_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CalibrationNode>());
    rclcpp::shutdown();
    return 0;
}
