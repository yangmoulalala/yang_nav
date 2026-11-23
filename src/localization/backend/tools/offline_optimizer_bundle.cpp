/** Adapt yang_nav bundles to/from HWSentryNav26's batch optimizer. */
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "arguments.hpp"
#include "backend/directory.hpp"
#include "backend/keyframe_map.hpp"
#include "bundle.hpp"

namespace fs = std::filesystem;
using namespace backend;

namespace {
struct Options {
    std::string input, output, results;
    bool help = false;
};

void writePose(std::ostream& out, const Eigen::Matrix4d& pose) {
    const Eigen::Quaterniond q(pose.block<3, 3>(0, 0));
    out << "se3(" << pose(0, 3) << ',' << pose(1, 3) << ',' << pose(2, 3) << ','
        << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ")\n";
}

std::vector<Eigen::Matrix4d> readPoses(const fs::path& path, std::size_t count) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    std::vector<Eigen::Matrix4d> poses;
    std::string line;
    while (std::getline(in, line)) {
        if (line.size() < 6 || line.substr(0, 4) != "se3(" || line.back() != ')')
            throw std::runtime_error("invalid optimized pose row");
        line = line.substr(4, line.size() - 5);
        for (char& c : line) if (c == ',') c = ' ';
        std::istringstream row(line);
        double x, y, z, qx, qy, qz, qw;
        std::string extra;
        if (!(row >> x >> y >> z >> qx >> qy >> qz >> qw) || (row >> extra))
            throw std::runtime_error("invalid optimized pose fields");
        const Eigen::Vector3d t(x, y, z);
        Eigen::Quaterniond q(qw, qx, qy, qz);
        if (!t.allFinite() || !q.coeffs().allFinite() || !std::isfinite(q.norm()) || q.norm() < 1e-9)
            throw std::runtime_error("nonfinite optimized pose");
        q.normalize();
        Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
        pose.block<3, 3>(0, 0) = q.toRotationMatrix();
        pose.block<3, 1>(0, 3) = t;
        poses.push_back(pose);
    }
    if (in.bad() || poses.size() != count)
        throw std::runtime_error("optimized pose count differs from snapshot");
    return poses;
}

void run(const Options& args) {
    tools::validateOutputLocation(args.input, args.output);
    if (!args.results.empty()) tools::validateOutputLocation(args.results, args.output);
    DirectoryTransaction output(args.output);
    std::string error;
    if (!output.prepare(&error)) throw std::runtime_error(error);
    KeyframeMap map;
    if (!map.load(args.input)) throw std::runtime_error("cannot load input map bundle");
    auto frames = map.snapshot();
    const auto descriptors = tools::loadDescriptorConfigs(args.input);
    const auto staged = output.stagingPath();
    if (args.results.empty()) {
        // Freeze one consistent native snapshot. Raw-format PCD names point to its
        // clouds, so optimization and import see exactly the same geometry.
        tools::writeStagedBundle(staged.string(), frames, descriptors);
        std::ofstream poses(staged / "poses.txt");
        poses << std::setprecision(17);
        for (std::size_t i = 0; i < frames.size(); ++i) {
            writePose(poses, frames[i].optimized_pose);
            std::ostringstream name;
            name << "clouds/" << std::setw(6) << std::setfill('0') << i << ".pcd";
            fs::create_hard_link(staged / name.str(), staged / ("frame_" + std::to_string(i) + ".pcd"));
        }
        poses.flush();
        if (!poses) throw std::runtime_error("cannot write input poses");
    } else {
        const auto poses = readPoses(fs::path(args.results) / "optimized_poses.txt", frames.size());
        std::vector<Keyframe> filtered;
        std::size_t removed = 0, total = 0;
        std::ofstream mapping(staged / "optimization.txt");
        mapping << "# original_index output_index retained_points input_points\n";
        for (std::size_t i = 0; i < frames.size(); ++i) {
            const auto cloud = frames[i].cloud.load();
            std::ifstream mask(fs::path(args.results) / ("retained_" + std::to_string(i) + ".txt"));
            if (!mask) throw std::runtime_error("missing retained point mask");
            Cloud::Ptr kept(new Cloud);
            std::size_t index = 0, previous = 0;
            std::string row;
            while (std::getline(mask, row)) {
                std::istringstream value(row);
                std::string extra;
                if (!(value >> index) || (value >> extra) || index >= cloud->size() ||
                    (!kept->empty() && index <= previous))
                    throw std::runtime_error("invalid retained point index");
                kept->push_back((*cloud)[index]);
                previous = index;
            }
            if (mask.bad()) throw std::runtime_error("cannot read retained point mask");
            total += cloud->size();
            removed += cloud->size() - kept->size();
            const int new_index = kept->empty() ? -1 : static_cast<int>(filtered.size());
            mapping << i << ' ' << new_index << ' ' << kept->size() << ' ' << cloud->size() << '\n';
            if (kept->empty()) continue;
            frames[i].index = new_index;
            frames[i].optimized_pose = poses[i];
            frames[i].odom_pose = poses[i];
            frames[i].cloud = CloudBlock(kept);
            filtered.push_back(frames[i]);
        }
        if (filtered.empty()) throw std::runtime_error("raycasting removed all keyframes");
        tools::writeStagedBundle(staged.string(), filtered, descriptors);
        // map.pcd is the global display/terrain cloud after sampling and MLS;
        // local clouds retain measured geometry and original intensity.
        fs::copy_file(fs::path(args.results) / "optimized_map.pcd", staged / "map.pcd");
        fs::copy_file(fs::path(args.results) / "optimized_poses.txt", staged / "optimized_poses.txt");
        fs::copy_file(fs::path(args.results) / "optimizer_params.yaml", staged / "optimizer_params.yaml");
        mapping << "# removed_points " << removed << '/' << total << '\n';
        mapping.flush();
        if (!mapping) throw std::runtime_error("cannot write optimization report");
        std::cout << "Optimized bundle: " << filtered.size() << " keyframes, removed "
                  << removed << '/' << total << " local points\n";
    }
    if (!output.commit(&error)) throw std::runtime_error(error);
    std::cout << "Saved " << args.output << '\n';
}
}  // namespace

int main(int argc, char** argv) {
    try {
        Options args;
        tools::cli::Arguments options;
        options.value({"--in"}, args.input);
        options.value({"--out"}, args.output);
        options.value({"--results"}, args.results);
        const bool valid = options.parse(argc, argv, args.help);
        if (!valid || args.help || args.input.empty() || args.output.empty()) {
            std::cout << "Usage: offline_optimizer_bundle --in BUNDLE --out NEW_DIR [--results RESULTS]\n"
                         "Without --results: export frozen native/raw snapshot.\n"
                         "With --results: import optimized poses/masks, rebuild descriptors.\n";
            return valid && args.help ? 0 : 1;
        }
        run(args);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Optimizer bundle conversion failed: " << error.what() << '\n';
        return 1;
    }
}
