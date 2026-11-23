#include <rclcpp/rclcpp.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/surface/mls.h>

#include <gtsam/geometry/Pose3.h>

#include <gtsam_points/types/point_cloud_cpu.hpp>
#include <gtsam_points/ann/kdtree.hpp>

#include <offline_mapping_optimizer/pose_optimizer.hpp>
#include <offline_mapping_optimizer/raycasting_filter.hpp>
#include <offline_mapping_optimizer/voxel_key.hpp>
#include <filesystem>
#include <iomanip>
#include <unordered_set>

#include <regex>
#include <fstream>

#include <omp.h>

struct {
    int num_threads;

    bool enable_factor_graph_optimization;
    int k_neighbors;

    // Local submap optimization
    int local_submap_size;
    double local_first_frame_prior_precision;
    bool local_enable_odometry_between;
    double local_odom_between_sigma;
    bool local_enable_vgicp_factors;
    int local_keyframe_stride;
    int local_max_vgicp_pairs_per_keyframe;
    int local_vgicp_num_threads;
    double local_voxel_resolution;
    int local_voxelmap_levels;
    double local_voxelmap_scaling_factor;
    double local_min_overlap;
    int local_min_frame_points;
    bool local_enable_optimization;
    int local_max_iterations;

    // Global submap graph optimization
    bool global_enable_optimization;
    int global_max_iterations;
    double global_first_submap_prior_precision;
    bool global_enable_adjacent_vgicp;
    int global_vgicp_num_threads;
    bool global_enable_odometry_between;
    double global_odom_between_sigma;
    bool global_enable_loop_closures;
    double global_max_loop_distance;
    double global_min_loop_overlap;
    int global_max_loop_edges_per_submap;
    double global_voxel_resolution;
    int global_voxelmap_levels;
    double global_voxelmap_scaling_factor;

    bool enable_raycasting_filter;
    double voxel_resolution;
    int pass_through_threshold;

    bool enable_downsample;
    double downsample_leaf_size;

    bool enable_radius_outlier_removal;
    double radius_search;
    int min_neighbors_in_radius;

    bool enable_statistical_outlier_removal;
    int mean_k;
    double std_dev_mul_thresh;

    bool enable_mls_smoothing;
    double search_radius;
    int polynomial_order;
} node_params;

inline Eigen::Array3i fast_floor(const Eigen::Vector3d& pt) {
    Eigen::Array3d arr = pt.array();
    Eigen::Array3i ncoord = arr.cast<int>();
    return ncoord - (arr < ncoord.cast<double>()).cast<int>();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_sampling(const pcl::PointCloud<pcl::PointXYZ>& input, double leaf_size) {
    if (input.empty()) {
        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    }

    const double inv_leaf_size = 1.0 / leaf_size;

    constexpr std::uint64_t invalid_coord = std::numeric_limits<std::uint64_t>::max();
    constexpr int coord_bit_size = 21; // 21 bits per axis → 63 bits total
    constexpr std::uint64_t coord_bit_mask = (1ULL << coord_bit_size) - 1;
    constexpr int coord_offset = 1 << (coord_bit_size - 1); // to make coords non-negative

    std::vector<std::pair<std::uint64_t, size_t>> coord_pt;
    coord_pt.reserve(input.size());

    for (size_t i = 0; i < input.size(); i++) {
        const auto& p = input.points[i];
        // Skip NaN points
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            coord_pt.emplace_back(invalid_coord, i);
            continue;
        }

        Eigen::Vector3d pt(p.x, p.y, p.z);
        Eigen::Array3i coord = fast_floor(pt * inv_leaf_size) + coord_offset;

        if ((coord < 0).any() || (coord > static_cast<int>(coord_bit_mask)).any()) {
            std::cerr << "Warning: voxel coordinate out of range!" << std::endl;
            coord_pt.emplace_back(invalid_coord, i);
            continue;
        }

        // Pack x, y, z into uint64_t: [unused(1b)][z(21b)][y(21b)][x(21b)]
        std::uint64_t bits = (static_cast<std::uint64_t>(coord[0] & coord_bit_mask) << (0 * coord_bit_size))
            | (static_cast<std::uint64_t>(coord[1] & coord_bit_mask) << (1 * coord_bit_size))
            | (static_cast<std::uint64_t>(coord[2] & coord_bit_mask) << (2 * coord_bit_size));

        coord_pt.emplace_back(bits, i);
    }

    // Sort by voxel key
    std::sort(coord_pt.begin(), coord_pt.end(), [](const auto& a, const auto& b) { return a.first < b.first; });

    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    output->reserve(input.size());

    size_t i = 0;
    while (i < coord_pt.size()) {
        if (coord_pt[i].first == invalid_coord) {
            i++;
            continue;
        }

        std::uint64_t current_voxel = coord_pt[i].first;
        Eigen::Vector3d sum(0, 0, 0);
        int count = 0;

        // Accumulate all points in the same voxel
        while (i < coord_pt.size() && coord_pt[i].first == current_voxel) {
            const auto& p = input.points[coord_pt[i].second];
            sum += Eigen::Vector3d(p.x, p.y, p.z);
            count++;
            i++;
        }

        // Compute centroid
        sum /= static_cast<double>(count);
        output->push_back(
            pcl::PointXYZ(static_cast<float>(sum.x()), static_cast<float>(sum.y()), static_cast<float>(sum.z()))
        );
    }

    return output;
}

rclcpp::Logger get_logger() {
    return rclcpp::get_logger("offline_mapping_optimizer");
}

gtsam::Pose3 parse_pose(const std::string& line) {
    std::regex re(R"(se3\(([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+),([^,]+)\))");
    std::smatch match;
    if (std::regex_search(line, match, re)) {
        double x = std::stod(match[1]);
        double y = std::stod(match[2]);
        double z = std::stod(match[3]);
        double qx = std::stod(match[4]);
        double qy = std::stod(match[5]);
        double qz = std::stod(match[6]);
        double qw = std::stod(match[7]);
        Eigen::Quaterniond q(qw, qx, qy, qz);
        if (!Eigen::Vector3d(x, y, z).allFinite() || !q.coeffs().allFinite() ||
            !std::isfinite(q.norm()) || q.norm() < 1e-9)
            throw std::runtime_error("Invalid pose: " + line);
        q.normalize();
        return gtsam::Pose3(gtsam::Rot3(q), gtsam::Point3(x, y, z));
    }
    throw std::runtime_error("Failed to parse pose: " + line);
}

std::vector<int> find_neighbors(const Eigen::Vector4d* points, const int num_points, const int k) {
    gtsam_points::KdTree tree(points, num_points);
    std::vector<int> neighbors(num_points * k);
    const auto perpoint_task = [&](int i) {
        std::vector<size_t> k_indices(k);
        std::vector<double> k_sq_dists(k);
        tree.knn_search(points[i].data(), k, k_indices.data(), k_sq_dists.data());
        std::copy(k_indices.begin(), k_indices.end(), neighbors.begin() + i * k);
    };
#pragma omp parallel for num_threads(node_params.num_threads) schedule(guided, 8)
    for (int i = 0; i < num_points; i++) {
        perpoint_task(i);
    }
    return neighbors;
}

Eigen::Matrix4d regularize(const Eigen::Matrix4d& cov, Eigen::Vector3d* eigenvalues, Eigen::Matrix3d* eigenvectors) {
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig;
    eig.computeDirect(cov.block<3, 3>(0, 0));
    if (eigenvalues)
        *eigenvalues = eig.eigenvalues();
    if (eigenvectors)
        *eigenvectors = eig.eigenvectors();
    Eigen::Vector3d values(1e-3, 1.0, 1.0);
    Eigen::Matrix4d c = Eigen::Matrix4d::Zero();
    c.block<3, 3>(0, 0) = eig.eigenvectors() * values.asDiagonal() * eig.eigenvectors().transpose();
    return c;
}

void estimate_cov(
    const std::vector<Eigen::Vector4d>& points,
    const std::vector<int>& neighbors,
    const int k_neighbors,
    std::vector<Eigen::Vector4d>& normals,
    std::vector<Eigen::Matrix4d>& covs
) {
    if (points.empty()) {
        return;
    }

    const int k_correspondences = neighbors.size() / points.size();
    assert(k_correspondences * points.size() == neighbors.size());
    assert(k_neighbors <= k_correspondences);

    // Precompute pt * pt.transpose()
    std::vector<Eigen::Matrix4d> pt_cross(points.size());
#pragma omp parallel for num_threads(node_params.num_threads) schedule(guided, 64)
    for (size_t i = 0; i < points.size(); i++) {
        pt_cross[i] = points[i] * points[i].transpose();
    }

    normals.resize(points.size());
    covs.resize(points.size());

    const auto calc_cov = [&](int i) {
        Eigen::Vector4d sum_points = Eigen::Vector4d::Zero();
        Eigen::Matrix4d sum_cross = Eigen::Matrix4d::Zero();

        const int begin = k_correspondences * i;
        for (int j = 0; j < k_neighbors; j++) {
            const int index = neighbors[begin + j];
            sum_points += points[index];
            sum_cross += pt_cross[index];
        }

        const Eigen::Vector4d mean = sum_points / k_neighbors;
        const Eigen::Matrix4d cov = (sum_cross - mean * sum_points.transpose()) / k_neighbors;

        Eigen::Matrix3d eigenvectors;
        covs[i] = regularize(cov, nullptr, &eigenvectors);
        covs[i](3, 3) = 0.0;

        normals[i] << eigenvectors.col(0), 0.0;
        if (points[i].dot(normals[i]) > 0.0) {
            normals[i] = -normals[i];
        }
    };

// Calculate covariances
#pragma omp parallel for num_threads(node_params.num_threads) schedule(guided, 8)
    for (size_t i = 0; i < points.size(); i++) {
        calc_cov(i);
    }
}

gtsam_points::PointCloudCPU::Ptr pcl_to_gtsam_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
    std::vector<Eigen::Vector4d> points;
    points.reserve(pcl_cloud ? pcl_cloud->size() : 0);
    if (pcl_cloud) {
        for (const auto& pt: *pcl_cloud) {
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
                continue;
            }
            points.emplace_back(pt.x, pt.y, pt.z, 1.0);
        }
    }

    // IMPORTANT: Ensure the point cloud owns its memory.
    // The PointCloudCPU(pointer, n) constructor may not deep-copy depending on build;
    // using add_points() guarantees points are stored in points_storage.
    auto frame = std::make_shared<gtsam_points::PointCloudCPU>();
    frame->add_points(points);
    if (frame->size() == 0) {
        return frame;
    }

    // Clamp k to a safe range for knn/covariance estimation.
    // VGICP is sensitive to invalid covariances; for tiny frames, skip cov estimation.
    int k = std::max(1, node_params.k_neighbors);
    if (frame->size() <= 2) {
        return frame;
    }
    k = std::min<int>(k, frame->size() - 1);

    std::vector<int> neighbors = find_neighbors(frame->points, frame->size(), k);
    std::vector<Eigen::Vector4d> normals;
    std::vector<Eigen::Matrix4d> covs;
    estimate_cov(points, neighbors, k, normals, covs);
    frame->add_covs(covs);
    frame->add_normals(normals);

    return frame;
}

int run(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("offline_mapping_optimizer");
    node_params = {
        (int)node->declare_parameter<int>("num_threads"),
        node->declare_parameter<bool>("enable_factor_graph_optimization"),
        (int)node->declare_parameter<int>("k_neighbors"),

        // Local submap optimization
        (int)node->declare_parameter<int>("local_submap_size"),
        node->declare_parameter<double>("local_first_frame_prior_precision"),
        node->declare_parameter<bool>("local_enable_odometry_between"),
        node->declare_parameter<double>("local_odom_between_sigma"),
        node->declare_parameter<bool>("local_enable_vgicp_factors"),
        (int)node->declare_parameter<int>("local_keyframe_stride"),
        (int)node->declare_parameter<int>("local_max_vgicp_pairs_per_keyframe"),
        (int)node->declare_parameter<int>("local_vgicp_num_threads"),
        node->declare_parameter<double>("local_voxel_resolution"),
        (int)node->declare_parameter<int>("local_voxelmap_levels"),
        node->declare_parameter<double>("local_voxelmap_scaling_factor"),
        node->declare_parameter<double>("local_min_overlap"),
        (int)node->declare_parameter<int>("local_min_frame_points"),
        node->declare_parameter<bool>("local_enable_optimization"),
        (int)node->declare_parameter<int>("local_max_iterations"),

        // Global submap graph optimization
        node->declare_parameter<bool>("global_enable_optimization"),
        (int)node->declare_parameter<int>("global_max_iterations"),
        node->declare_parameter<double>("global_first_submap_prior_precision"),
        node->declare_parameter<bool>("global_enable_adjacent_vgicp"),
        (int)node->declare_parameter<int>("global_vgicp_num_threads"),
        node->declare_parameter<bool>("global_enable_odometry_between"),
        node->declare_parameter<double>("global_odom_between_sigma"),
        node->declare_parameter<bool>("global_enable_loop_closures"),
        node->declare_parameter<double>("global_max_loop_distance"),
        node->declare_parameter<double>("global_min_loop_overlap"),
        (int)node->declare_parameter<int>("global_max_loop_edges_per_submap"),
        node->declare_parameter<double>("global_voxel_resolution"),
        (int)node->declare_parameter<int>("global_voxelmap_levels"),
        node->declare_parameter<double>("global_voxelmap_scaling_factor"),

        node->declare_parameter<bool>("enable_raycasting_filter"),
        node->declare_parameter<double>("voxel_resolution"),
        (int)node->declare_parameter<int>("pass_through_threshold"),
        node->declare_parameter<bool>("enable_downsample"),
        node->declare_parameter<double>("downsample_leaf_size"),
        node->declare_parameter<bool>("enable_radius_outlier_removal"),
        node->declare_parameter<double>("radius_search"),
        (int)node->declare_parameter<int>("min_neighbors_in_radius"),
        node->declare_parameter<bool>("enable_statistical_outlier_removal"),
        (int)node->declare_parameter<int>("mean_k"),
        node->declare_parameter<double>("std_dev_mul_thresh"),
        node->declare_parameter<bool>("enable_mls_smoothing"),
        node->declare_parameter<double>("search_radius"),
        (int)node->declare_parameter<int>("polynomial_order")
    };
    std::string data_path = node->declare_parameter<std::string>("data_path");
    bool use_cuda_raycasting = node->declare_parameter<bool>("use_cuda_raycasting");
    const auto lidar_offset = node->declare_parameter<std::vector<double>>("lidar_origin");
    if (lidar_offset.size() != 3)
        throw std::invalid_argument("lidar_origin must contain three coordinates in the local cloud frame");
    const Eigen::Vector3d lidar_origin(lidar_offset[0], lidar_offset[1], lidar_offset[2]);
    if (!lidar_origin.allFinite()) throw std::invalid_argument("Nonfinite lidar_origin");
    std::string output_path = node->declare_parameter<std::string>("output_path");
    const auto positive = [](double v) { return std::isfinite(v) && v > 0; };
    const auto overlap = [](double v) { return std::isfinite(v) && v >= 0 && v <= 1; };
    if (node_params.num_threads < 1 || node_params.k_neighbors < 3 ||
        node_params.local_submap_size < 1 || node_params.local_keyframe_stride < 1 ||
        node_params.local_max_iterations < 1 || node_params.global_max_iterations < 1 ||
        node_params.local_vgicp_num_threads < 1 || node_params.global_vgicp_num_threads < 1 ||
        node_params.local_voxelmap_levels < 1 || node_params.global_voxelmap_levels < 1 ||
        !positive(node_params.local_voxel_resolution) || !positive(node_params.global_voxel_resolution) ||
        !positive(node_params.local_first_frame_prior_precision) ||
        !positive(node_params.global_first_submap_prior_precision) ||
        !positive(node_params.local_odom_between_sigma) || !positive(node_params.global_odom_between_sigma) ||
        !positive(node_params.local_voxelmap_scaling_factor) ||
        !positive(node_params.global_voxelmap_scaling_factor) ||
        !positive(node_params.global_max_loop_distance) ||
        !overlap(node_params.local_min_overlap) || !overlap(node_params.global_min_loop_overlap) ||
        node_params.local_max_vgicp_pairs_per_keyframe < 0 ||
        node_params.global_max_loop_edges_per_submap < 0 || node_params.local_min_frame_points < 0 ||
        !positive(node_params.voxel_resolution) || node_params.pass_through_threshold < 1 ||
        !positive(node_params.downsample_leaf_size) || !positive(node_params.radius_search) ||
        node_params.min_neighbors_in_radius < 1 || node_params.mean_k < 1 ||
        !positive(node_params.std_dev_mul_thresh) || !positive(node_params.search_radius) ||
        node_params.polynomial_order < 1)
        throw std::invalid_argument("Invalid optimizer parameters: check scales, counts and overlaps");
    if (data_path.empty() || output_path.empty())
        throw std::invalid_argument("data_path and output_path are required");
    const auto input_dir = std::filesystem::canonical(data_path);
    const auto output_dir = std::filesystem::weakly_canonical(output_path);
    const auto relative = output_dir.lexically_relative(input_dir);
    if (relative.empty() || *relative.begin() != ".." || std::filesystem::exists(output_dir))
        throw std::invalid_argument("output_path must be new and outside data_path");
    std::vector<gtsam::Pose3> poses;
    std::vector<gtsam_points::PointCloudCPU::Ptr> frames;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> pcl_frames;

    // Load poses
    std::string poses_path = data_path + "/poses.txt";
    std::ifstream poses_file(poses_path);
    if (!poses_file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to open %s", poses_path.c_str());
        return 1;
    }
    std::string line;
    while (std::getline(poses_file, line)) {
        if (line.empty())
            continue;
        try {
            poses.push_back(parse_pose(line));
        } catch (const std::exception& e) {
            throw std::runtime_error("Invalid poses.txt row: " + line);
        }
    }
    if (poses_file.bad() || poses.empty())
        throw std::runtime_error("Empty or unreadable poses.txt");
    RCLCPP_INFO(get_logger(), "Loaded %zu poses", poses.size());

    // Load frames
    for (size_t i = 0; i < poses.size(); i++) {
        std::string pcd_path = data_path + "/frame_" + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        if (pcl::io::loadPCDFile(pcd_path, *cloud) == -1) {
            RCLCPP_ERROR(get_logger(), "Failed to load %s", pcd_path.c_str());
            return 1;
        }
        if (cloud->size() < static_cast<std::size_t>(node_params.k_neighbors))
            throw std::runtime_error("Too few points for covariance estimation: " + pcd_path);
        for (const auto& p : *cloud)
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                throw std::runtime_error("Non-finite point in " + pcd_path);
        pcl_frames.push_back(cloud);
        frames.push_back(pcl_to_gtsam_points(cloud));
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu frames", frames.size());

    if (poses.size() != frames.size()) {
        RCLCPP_ERROR(get_logger(), "Number of poses and frames do not match");
        return 1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr merged = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    if (node_params.enable_factor_graph_optimization) {
        offline_mapping_optimizer::OfflineMappingOptimizationParams opt;
        opt.num_threads = node_params.num_threads;

        opt.local.submap_size = node_params.local_submap_size;
        opt.local.first_frame_prior_precision = node_params.local_first_frame_prior_precision;
        opt.local.enable_odometry_between = node_params.local_enable_odometry_between;
        opt.local.odom_between_sigma = node_params.local_odom_between_sigma;
        opt.local.enable_vgicp_factors = node_params.local_enable_vgicp_factors;
        opt.local.keyframe_stride = node_params.local_keyframe_stride;
        opt.local.max_vgicp_pairs_per_keyframe = node_params.local_max_vgicp_pairs_per_keyframe;
        opt.local.vgicp_num_threads = node_params.local_vgicp_num_threads;
        opt.local.voxel_resolution = node_params.local_voxel_resolution;
        opt.local.voxelmap_levels = node_params.local_voxelmap_levels;
        opt.local.voxelmap_scaling_factor = node_params.local_voxelmap_scaling_factor;
        opt.local.min_overlap = node_params.local_min_overlap;
        opt.local.min_frame_points = node_params.local_min_frame_points;
        opt.local.enable_optimization = node_params.local_enable_optimization;
        opt.local.max_iterations = node_params.local_max_iterations;

        opt.global.enable_optimization = node_params.global_enable_optimization;
        opt.global.max_iterations = node_params.global_max_iterations;
        opt.global.first_submap_prior_precision = node_params.global_first_submap_prior_precision;
        opt.global.enable_adjacent_vgicp = node_params.global_enable_adjacent_vgicp;
        opt.global.vgicp_num_threads = node_params.global_vgicp_num_threads;
        opt.global.enable_odometry_between = node_params.global_enable_odometry_between;
        opt.global.odom_between_sigma = node_params.global_odom_between_sigma;
        opt.global.enable_loop_closures = node_params.global_enable_loop_closures;
        opt.global.max_loop_distance = node_params.global_max_loop_distance;
        opt.global.min_loop_overlap = node_params.global_min_loop_overlap;
        opt.global.max_loop_edges_per_submap = node_params.global_max_loop_edges_per_submap;
        opt.global.voxel_resolution = node_params.global_voxel_resolution;
        opt.global.voxelmap_levels = node_params.global_voxelmap_levels;
        opt.global.voxelmap_scaling_factor = node_params.global_voxelmap_scaling_factor;

        RCLCPP_INFO(get_logger(), "Optimizing poses...");
        poses = offline_mapping_optimizer::optimize_poses_submap_graph(poses, frames, opt, get_logger());
        RCLCPP_INFO(get_logger(), "Pose optimization done");
    }

    // Merge point clouds
    for (size_t i = 0; i < poses.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        Eigen::Matrix4f T = poses[i].matrix().cast<float>();
        pcl::transformPointCloud(*pcl_frames[i], *transformed, T);
        *merged += *transformed;
    }

    // Dynamic Object Removal
    if (node_params.enable_raycasting_filter) {
        merged = offline_mapping_optimizer::remove_dynamic_objects_raycasting(
            merged,
            poses,
            pcl_frames,
            node_params.voxel_resolution,
            node_params.pass_through_threshold,
            node_params.num_threads,
            use_cuda_raycasting,
            lidar_origin,
            get_logger()
        );
    }

    // Masks precede sampling/smoothing, preserving original local XYZ/I geometry.
    if (!std::filesystem::create_directories(output_dir))
        throw std::runtime_error("Cannot create new result directory");
    std::unordered_set<VoxelKey, VoxelKeyHash> kept;
    const double inv_res = 1.0 / node_params.voxel_resolution;
    if (node_params.enable_raycasting_filter)
        for (const auto& p : *merged)
            kept.insert(voxel_key_from_xyz(p.x, p.y, p.z, inv_res));
    for (std::size_t i = 0; i < pcl_frames.size(); ++i) {
        pcl::PointCloud<pcl::PointXYZ> world;
        const Eigen::Matrix4f transform = poses[i].matrix().cast<float>();
        pcl::transformPointCloud(*pcl_frames[i], world, transform);
        std::ofstream mask(output_dir / ("retained_" + std::to_string(i) + ".txt"));
        for (std::size_t j = 0; j < world.size(); ++j) {
            const auto& p = world[j];
            if (!node_params.enable_raycasting_filter || kept.count(
                    voxel_key_from_xyz(p.x, p.y, p.z, inv_res)))
                mask << j << '\n';
        }
        mask.flush();
        if (!mask) throw std::runtime_error("Cannot write retained point mask");
    }

    // Downsample
    if (node_params.enable_downsample) {
        RCLCPP_INFO(get_logger(), "Downsampling...");
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled = voxelgrid_sampling(*merged, node_params.downsample_leaf_size);
        RCLCPP_INFO(get_logger(), "Downsampling done. Kept %zu / %zu points", downsampled->size(), merged->size());
        *merged = *downsampled;
    }

    // Remove radius outliers
    if (node_params.enable_radius_outlier_removal) {
        RCLCPP_INFO(get_logger(), "Removing radius outliers...");
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
        ror.setInputCloud(merged);
        ror.setRadiusSearch(node_params.radius_search);
        ror.setMinNeighborsInRadius(node_params.min_neighbors_in_radius);
        ror.filter(*filtered);
        RCLCPP_INFO(
            get_logger(),
            "Radius outlier removal done. Kept %zu / %zu points",
            filtered->size(),
            merged->size()
        );
        *merged = *filtered;
    }

    // Remove statistical outliers
    if (node_params.enable_statistical_outlier_removal) {
        RCLCPP_INFO(get_logger(), "Removing statistical outliers...");
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(merged);
        sor.setMeanK(node_params.mean_k);
        sor.setStddevMulThresh(node_params.std_dev_mul_thresh);
        sor.filter(*filtered);
        RCLCPP_INFO(
            get_logger(),
            "Statistical outlier removal done. Kept %zu / %zu points",
            filtered->size(),
            merged->size()
        );
        *merged = *filtered;
    }

    // MLS smoothing
    if (node_params.enable_mls_smoothing) {
        RCLCPP_INFO(get_logger(), "Applying MLS smoothing...");
        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothed = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;
        mls.setInputCloud(merged);
        mls.setSearchMethod(tree);
        mls.setSearchRadius(node_params.search_radius);
        mls.setPolynomialOrder(node_params.polynomial_order);
        mls.process(*smoothed);
        RCLCPP_INFO(get_logger(), "MLS smoothing done.");
        *merged = *smoothed;
    }

    std::string pcd_output_path = (output_dir / "optimized_map.pcd").string();
    if (merged->empty() || pcl::io::savePCDFileBinary(pcd_output_path, *merged) != 0)
        throw std::runtime_error("Cannot save nonempty optimized map");
    RCLCPP_INFO(get_logger(), "Saved optimized map to %s", pcd_output_path.c_str());

    // Save optimized poses
    std::string pose_output_path = (output_dir / "optimized_poses.txt").string();
    std::ofstream out_poses(pose_output_path);
    out_poses << std::setprecision(17);
    for (size_t i = 0; i < poses.size(); i++) {
        const gtsam::Pose3 pose = poses[i];
        auto q = pose.rotation().toQuaternion();
        auto t = pose.translation();
        // se3(x,y,z,qx,qy,qz,qw)
        out_poses << "se3(" << t.x() << "," << t.y() << "," << t.z() << "," << q.x() << "," << q.y() << "," << q.z()
                  << "," << q.w() << ")" << std::endl;
    }
    out_poses.flush();
    if (!out_poses) throw std::runtime_error("Cannot write optimized poses");
    std::ofstream config_out(output_dir / "optimizer_params.yaml");
    config_out << node->get_name() << ":\n  ros__parameters:\n";
    for (const auto& name : node->list_parameters({}, 10).names)
        config_out << "    " << name << ": " << node->get_parameter(name).value_to_string() << '\n';
    config_out.flush();
    if (!config_out) throw std::runtime_error("Cannot write parameter report");
    RCLCPP_INFO(get_logger(), "Saved optimized poses to %s", pose_output_path.c_str());
    rclcpp::shutdown();
    return 0;
}

int main(int argc, char* argv[]) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Offline optimization failed: " << e.what() << '\n';
        if (rclcpp::ok()) rclcpp::shutdown();
        return 1;
    }
}
