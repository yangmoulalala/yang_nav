// Add a collision-decreasing escape prefix before planning normally with Smac.
// The shared costmap is never cleared, masked or modified by this wrapper.
#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_behaviors/observed_approach.hpp"
#include "nav_behaviors/overlap_escape.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace nav_behaviors
{
class StartAwarePlanner : public nav2_core::GlobalPlanner
{
public:
  StartAwarePlanner() : loader_("nav2_core", "nav2_core::GlobalPlanner") {}
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map) override
  {
    auto node = parent.lock(); logger_ = node->get_logger(); map_ = map;
    nav2_util::declare_parameter_if_not_declared(node, name + ".escape_max_distance", rclcpp::ParameterValue(1.5));
    node->get_parameter(name + ".escape_max_distance", max_distance_);
    if (max_distance_ <= 0.0 || max_distance_ > 2.0) {
      throw std::runtime_error("StartAwarePlanner: escape_max_distance must be in (0,2]");
    }
    nav2_util::declare_parameter_if_not_declared(node, name + ".approach_occupied_goal", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".approach_occupied_goal", approach_goal_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".approach_unknown_goal", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".approach_unknown_goal", approach_unknown_goal_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".require_observed_path", rclcpp::ParameterValue(false));
    node->get_parameter(name + ".require_observed_path", require_observed_path_);
    require_observed_path_ = require_observed_path_ || approach_unknown_goal_;
    // Same parameter the inner Smac planner reads; unknown cells are passable when it is set.
    nav2_util::declare_parameter_if_not_declared(node, name + ".allow_unknown", rclcpp::ParameterValue(true));
    node->get_parameter(name + ".allow_unknown", allow_unknown_);
    nav2_util::declare_parameter_if_not_declared(node, name + ".approach_clearance", rclcpp::ParameterValue(0.20));
    node->get_parameter(name + ".approach_clearance", approach_clearance_);
    if (approach_clearance_ < 0.0 || approach_clearance_ > 1.0) {
      throw std::runtime_error("StartAwarePlanner: approach_clearance must be in [0,1]");
    }
    inner_ = loader_.createSharedInstance("nav2_smac_planner::SmacPlanner2D");
    inner_->configure(parent, name, tf, map);
  }
  void cleanup() override {inner_->cleanup(); inner_.reset(); map_.reset();}
  void activate() override {inner_->activate();}
  void deactivate() override {inner_->deactivate();}
  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> canceled) override
  {
    try {
      return planTowardsGoal(start, goal, canceled);
    } catch (const nav2_core::NoValidPathCouldBeFound &) {
      if (approach_goal_) {
        if (auto escape = planEscapeOnly(start, goal, canceled)) {return std::move(*escape);}
      }
      throw;
    }
  }

private:
  static Pose2 toPose2(const geometry_msgs::msg::PoseStamped & pose)
  {
    const auto & q = pose.pose.orientation;
    return {pose.pose.position.x, pose.pose.position.y,
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
  }

  static std::vector<geometry_msgs::msg::PoseStamped> translationPoses(
    const geometry_msgs::msg::PoseStamped & start, const std::vector<Pose2> & points)
  {
    std::vector<geometry_msgs::msg::PoseStamped> poses;
    poses.reserve(points.size());
    for (const auto & point : points) {
      auto pose = start;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      poses.push_back(pose);
    }
    return poses;
  }

  nav_msgs::msg::Path planTowardsGoal(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const std::function<bool()> & canceled)
  {
    if (approach_unknown_goal_) {return planInObservedSpace(start, goal, canceled);}
    if (approach_goal_) {
      auto * costs = map_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costs->getMutex());
      unsigned gx, gy;
      if (!costs->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        throw nav2_core::GoalOutsideMapBounds("Goal is outside the global map");
      }
      const auto target = toPose2(goal);
      auto approach_fp = map_->getRobotFootprint();
      const auto contacts = footprintContacts(*costs, approach_fp, target);
      if (contacts.valid && (!contacts.cells.empty() ||
        costs->getCost(gx, gy) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
      {
        nav2_costmap_2d::padFootprint(approach_fp, approach_clearance_);
        auto usable = [&](const geometry_msgs::msg::PoseStamped & candidate) {
            unsigned mx, my;
            const auto pose = toPose2(candidate);
            return costs->worldToMap(pose.x, pose.y, mx, my) &&
                   costs->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
                   footprintFree(*costs, approach_fp, pose);
          };
        std::vector<geometry_msgs::msg::PoseStamped> candidates;
        if (approach_anchor_ && anchor_goal_ == goal && usable(*approach_anchor_)) {
          candidates.push_back(*approach_anchor_);
        }
        approach_anchor_.reset();
        const double toward = std::atan2(start.pose.position.y - target.y, start.pose.position.x - target.x);
        std::vector<geometry_msgs::msg::PoseStamped> far_side;
        for (double radius = 0.4; radius <= 2.001; radius += 0.2) {
          for (int ray = 0; ray < 16; ++ray) {
            const int offset = (ray + 1) / 2 * (ray % 2 ? 1 : -1);
            const double angle = toward + offset * 2 * kPi / 16;
            auto candidate = goal;
            candidate.pose.position.x = target.x + radius * std::cos(angle);
            candidate.pose.position.y = target.y + radius * std::sin(angle);
            if (usable(candidate)) {
              (std::abs(offset) <= 4 ? candidates : far_side).push_back(candidate);
            }
          }
        }
        candidates.insert(candidates.end(), far_side.begin(), far_side.end());
        lock.unlock();
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        unsigned attempts = 0;
        for (const auto & candidate : candidates) {
          if (attempts++ >= 12 || std::chrono::steady_clock::now() > deadline) {break;}
          try {
            auto path = planFromStart(start, candidate, canceled);
            if (path.poses.empty()) {continue;}
            const auto & end = path.poses.back();
            {
              std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> recheck(*costs->getMutex());
              if (!usable(end) || std::hypot(end.pose.position.x - target.x,
                end.pose.position.y - target.y) <= 0.20) {continue;}
            }
            approach_anchor_ = end;
            anchor_goal_ = goal;
            RCLCPP_INFO_THROTTLE(logger_, *map_->get_clock(), 2000,
              "Occupied goal: approach (%.2f, %.2f); original target remains pending",
              end.pose.position.x, end.pose.position.y);
            return path;
          } catch (const nav2_core::PlannerCancelled &) {throw;}
          catch (const nav2_core::PlannerException &) {}
        }
        throw nav2_core::NoValidPathCouldBeFound("No safe approach to occupied goal");
      }
    }
    approach_anchor_.reset();
    return planFromStart(start,goal,canceled);
  }

  // Copy the neighbourhood the detour search can reach, so the search never holds
  // the master costmap mutex that the update thread and the controller need.
  static std::shared_ptr<nav2_costmap_2d::Costmap2D> detourWindow(
    nav2_costmap_2d::Costmap2D & source, const Pose2 & robot, const Pose2 & target)
  {
    const double resolution = source.getResolution();
    const double reach = std::hypot(target.x - robot.x, target.y - robot.y);
    const double half = std::min(12.0, std::max(3.0, reach + 1.0));
    const auto limit = [](double value, int low, int high) {
        return std::max(low, std::min(high, static_cast<int>(value)));
      };
    const int size_x = static_cast<int>(source.getSizeInCellsX());
    const int size_y = static_cast<int>(source.getSizeInCellsY());
    const int min_x = limit(std::floor((robot.x - half - source.getOriginX()) / resolution), 0, size_x);
    const int max_x = limit(std::ceil((robot.x + half - source.getOriginX()) / resolution), min_x, size_x);
    const int min_y = limit(std::floor((robot.y - half - source.getOriginY()) / resolution), 0, size_y);
    const int max_y = limit(std::ceil((robot.y + half - source.getOriginY()) / resolution), min_y, size_y);
    if (max_x - min_x < 8 || max_y - min_y < 8) {return nullptr;}
    auto window = std::make_shared<nav2_costmap_2d::Costmap2D>(
        static_cast<unsigned>(max_x - min_x), static_cast<unsigned>(max_y - min_y), resolution,
        source.getOriginX() + min_x * resolution, source.getOriginY() + min_y * resolution,
        nav2_costmap_2d::NO_INFORMATION);
    const auto * data = source.getCharMap();
    for (int row = min_y; row < max_y; ++row) {
      std::copy_n(data + static_cast<size_t>(row) * source.getSizeInCellsX() + min_x,
        max_x - min_x,
        window->getCharMap() + static_cast<size_t>(row - min_y) * window->getSizeInCellsX());
    }
    return window;
  }

  nav_msgs::msg::Path planInObservedSpace(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const std::function<bool()> & canceled)
  {
    auto * costs = map_->getCostmap();
    const auto robot = toPose2(start);
    auto target = toPose2(goal);
    target.yaw = robot.yaw;
    if (!std::isfinite(target.x) || !std::isfinite(target.y)) {
      throw nav2_core::NoValidPathCouldBeFound("Goal coordinates must be finite");
    }
    const auto footprint = map_->getRobotFootprint();
    using MapLock = std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t>;
    std::string direct_failure;
    try {
      auto direct = planFromStart(start, goal, canceled);
      if (fallbacks_ > 0) {
        RCLCPP_INFO(logger_, "StartAwarePlanner: direct planning recovered after %d fallback plans",
          fallbacks_);
        fallbacks_ = 0;
      }
      return direct;
    } catch (const nav2_core::PlannerCancelled &) {
      throw;
    } catch (const nav2_core::StartOutsideMapBounds &) {
      throw;
    } catch (const nav2_core::StartOccupied &) {
      throw;
    } catch (const nav2_core::PlannerException & error) {
      direct_failure = error.what();
    }
    if (canceled && canceled()) {throw nav2_core::PlannerCancelled("Approach planning canceled");}
    std::vector<Pose2> approach;
    std::shared_ptr<nav2_costmap_2d::Costmap2D> window;
    MapLock lock(*costs->getMutex());
    unsigned goal_col, goal_row;
    if (costs->worldToMap(target.x, target.y, goal_col, goal_row) &&
      observedTranslationFree(*costs, footprint, robot, target))
    {
      const auto segments = std::max(1, static_cast<int>(std::ceil(
        std::hypot(target.x - robot.x, target.y - robot.y) / costs->getResolution())));
      std::vector<Pose2> points;
      points.reserve(segments + 1);
      for (int segment = 0; segment < segments; ++segment) {
        const double fraction = static_cast<double>(segment) / segments;
        points.push_back({robot.x + fraction * (target.x - robot.x),
          robot.y + fraction * (target.y - robot.y), robot.yaw});
      }
      points.push_back(target);
      nav_msgs::msg::Path path;
      path.header = start.header;
      path.poses = translationPoses(start, points);
      path.poses.back().pose = goal.pose;
      return path;
    }
    const bool unknown_goal = needsObservedApproach(*costs, footprint, target);
    approach = observedApproach(*costs, footprint, robot, target, approach_clearance_, !unknown_goal);
    window = detourWindow(*costs, robot, target);
    lock.unlock();
    // The detour search is the expensive step. Run it on a copy of the neighbourhood
    // so the costmap update thread and the controller are not queued behind it.
    std::vector<Pose2> detour;
    if (window) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);
      detour = observedDetour(*window, footprint, robot, target, approach_clearance_, [&] {
          return (canceled && canceled()) || std::chrono::steady_clock::now() >= deadline;
        });
    }
    if (canceled && canceled()) {throw nav2_core::PlannerCancelled("Approach planning canceled");}
    auto remaining = [&](const Pose2 & pose) {
        return std::hypot(pose.x - target.x, pose.y - target.y);
      };
    if (!detour.empty() && (approach.empty() || remaining(detour.back()) == 0.0 ||
      remaining(detour.back()) + 0.15 < remaining(approach.back())))
    {
      approach = std::move(detour);
    }
    ++fallbacks_;
    RCLCPP_WARN_THROTTLE(logger_, *map_->get_clock(), 5000,
      "StartAwarePlanner: observed-space fallback #%d; direct plan rejected: %s",
      fallbacks_, direct_failure.c_str());
    if (approach.empty()) {
      throw nav2_core::NoValidPathCouldBeFound(
              "No observed free approach toward the goal; direct planning: " + direct_failure);
    }
    nav_msgs::msg::Path path;
    path.header = start.header;
    path.poses = translationPoses(start, approach);
    const auto & end = approach.back();
    if (end.x == target.x && end.y == target.y) {path.poses.back().pose = goal.pose; return path;}
    RCLCPP_INFO_THROTTLE(logger_, *map_->get_clock(), 2000,
      "Unknown space: approach (%.2f, %.2f); original target remains pending", end.x, end.y);
    return path;
  }

  void validateObservedPath(const nav_msgs::msg::Path & path, const Pose2 & start,
    const Pose2 & target) const
  {
    auto * costs = map_->getCostmap();
    const auto footprint = map_->getRobotFootprint();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costs->getMutex());
    if (!observedPathFree(*costs, footprint, path, start, target)) {
      throw nav2_core::NoValidPathCouldBeFound("Mapping path lacks observed footprint clearance");
    }
  }

  nav_msgs::msg::Path planFromStart(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const std::function<bool()> & canceled)
  {
    const auto a = toPose2(start);
    const auto b = toPose2(goal);
    std::vector<EscapePath> escapes;
    {
      auto * map = map_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      unsigned gx, gy;
      if (!map->worldToMap(b.x, b.y, gx, gy)) {
        throw nav2_core::GoalOutsideMapBounds("Goal is outside the global map");
      }
      const auto fp = map_->getRobotFootprint();
      unsigned sx, sy;
      // With allow_unknown the start may already touch unexplored cells; only real obstacles
      // make it an occupied start.
      const bool unknown_start_allowed = allow_unknown_ && !require_observed_path_;
      const bool footprint_clear = unknown_start_allowed ?
        footprintFreeInUnknown(*map, fp, a) : footprintFree(*map, fp, a);
      if (footprint_clear && map->worldToMap(a.x, a.y, sx, sy) &&
        (unknown_start_allowed ? map->getCost(sx, sy) < nav2_costmap_2d::LETHAL_OBSTACLE :
        map->getCost(sx, sy) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) {
        lock.unlock();
        auto path = inner_->createPlan(start, goal, canceled);
        if (!path.poses.empty()) {path.poses.front() = start;}
        if (require_observed_path_) {
          validateObservedPath(path, a, b);
          auto endpoint = path.poses.back();
          if (endpoint.pose.position != goal.pose.position) {
            endpoint.pose = goal.pose;
            path.poses.push_back(endpoint);
          }
        }
        return path;
      }
      if (require_observed_path_) {
        if (!map->worldToMap(a.x, a.y, sx, sy)) {
          throw nav2_core::StartOutsideMapBounds("Mapping start is outside the global costmap");
        }
        if (!footprintFreeInUnknown(*map, fp, a) ||
          (map->getCost(sx, sy) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
          map->getCost(sx, sy) != nav2_costmap_2d::NO_INFORMATION))
        {
          throw nav2_core::StartOccupied("Mapping start is blocked by an obstacle or its inflation");
        }
        throw nav2_core::NoValidPathCouldBeFound("Mapping start lacks observed footprint clearance");
      }
      escapes = escapePaths(*map, fp, a, b, max_distance_);
    }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    size_t attempts = 0;
    for (const auto & escape : escapes) {
      if (attempts++ >= 8 || std::chrono::steady_clock::now() >= deadline) {break;}
      if (canceled && canceled()) {throw nav2_core::PlannerCancelled("Escape planning canceled");}
      auto free_start = start;
      const auto & end = escape.poses.back();
      free_start.pose.position.x = end.x; free_start.pose.position.y = end.y;
      try {
        auto suffix = inner_->createPlan(free_start, goal, canceled);
        // Smac may replace the start heading with a tangent. Keep the escape's actual heading;
        // zone_shaper will validate the full swept transition from here to the suffix.
        if (suffix.poses.empty()) {continue;}
        suffix.poses.front() = free_start;
        auto prefix = translationPoses(start, escape.poses);
        prefix.insert(prefix.end(), suffix.poses.begin() + 1, suffix.poses.end());
        suffix.poses = std::move(prefix);
        RCLCPP_INFO(logger_, "StartAwarePlanner: occupied start, %.2f m translation escape before normal plan",
          std::hypot(end.x - a.x, end.y - a.y));
        return suffix;
      } catch (const nav2_core::PlannerCancelled &) {throw;}
      catch (const nav2_core::PlannerException &) { /* Try another decreasing-contact exit. */ }
    }
    throw nav2_core::NoValidPathCouldBeFound("No collision-decreasing escape from occupied start");
  }

  std::optional<nav_msgs::msg::Path> planEscapeOnly(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, const std::function<bool()> & canceled)
  {
    if (canceled && canceled()) {throw nav2_core::PlannerCancelled("Escape planning canceled");}
    const auto robot = toPose2(start);
    const auto target = toPose2(goal);
    auto * costs = map_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costs->getMutex());
    const auto footprint = map_->getRobotFootprint();
    unsigned mx, my;
    if (!costs->worldToMap(robot.x, robot.y, mx, my) ||
      (footprintFree(*costs, footprint, robot) &&
      costs->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE))
    {
      return std::nullopt;
    }
    const auto exits = escapePaths(*costs, footprint, robot, target, max_distance_, 0.20);
    for (const auto & escape : exits) {
      const auto & end = escape.poses.back();
      if (std::hypot(end.x - target.x, end.y - target.y) <= 0.20) {continue;}
      nav_msgs::msg::Path path;
      path.header = start.header;
      path.poses = translationPoses(start, escape.poses);
      RCLCPP_INFO_THROTTLE(logger_, *map_->get_clock(), 2000,
        "Occupied start: escape only; original target remains pending");
      return path;
    }
    return std::nullopt;
  }

  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_;
  nav2_core::GlobalPlanner::Ptr inner_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  rclcpp::Logger logger_{rclcpp::get_logger("StartAwarePlanner")};
  double max_distance_{1.5};
  double approach_clearance_{0.20};
  std::optional<geometry_msgs::msg::PoseStamped> approach_anchor_;
  geometry_msgs::msg::PoseStamped anchor_goal_;
  bool approach_goal_{false};
  bool approach_unknown_goal_{false};
  bool require_observed_path_{false};
  bool allow_unknown_{true};
  int fallbacks_{0};
};
}  // namespace nav_behaviors
PLUGINLIB_EXPORT_CLASS(nav_behaviors::StartAwarePlanner, nav2_core::GlobalPlanner)
