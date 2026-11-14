// Add a collision-decreasing escape prefix before planning normally with Smac.
// The shared costmap is never cleared, masked or modified by this wrapper.
#include <chrono>
#include <algorithm>
#include <optional>
#include "nav2_costmap_2d/static_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "nav2_util/node_utils.hpp"
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
    if (approach_goal_) {
      auto * costs = map_->getCostmap();
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costs->getMutex());
      unsigned gx, gy;
      if (!costs->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        throw nav2_core::GoalOutsideMapBounds("Goal is outside the global map");
      }
      const auto & qg=goal.pose.orientation;
      const Pose2 target{goal.pose.position.x,goal.pose.position.y,
        std::atan2(2*(qg.w*qg.z+qg.x*qg.y),1-2*(qg.y*qg.y+qg.z*qg.z))};
      const auto fp=map_->getRobotFootprint();
      if (!footprintFree(*costs, fp, target) || costs->getCost(gx,gy)>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        // Static walls/out-of-map targets remain errors. Only live occupancy gets an approach path.
        bool static_free=false;
        for (const auto & plugin : *map_->getLayeredCostmap()->getPlugins()) {
          auto fixed=std::dynamic_pointer_cast<nav2_costmap_2d::StaticLayer>(plugin);
          if (fixed) {static_free=footprintFree(*fixed,fp,target);break;}
        }
        if (static_free) {
          // The approach endpoint is subject to the controller's position tolerance.
          // Reserve that settling space here, without enlarging the driving footprint
          // everywhere (which would make narrow tunnels impossible).
          auto approach_fp = fp;
          nav2_costmap_2d::padFootprint(approach_fp, approach_clearance_);
          auto usable = [&](const geometry_msgs::msg::PoseStamped & candidate) {
              unsigned mx, my;
              const Pose2 pose{candidate.pose.position.x, candidate.pose.position.y, target.yaw};
              return costs->worldToMap(pose.x, pose.y, mx, my) &&
                     costs->getCost(mx, my) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
                     footprintFree(*costs, approach_fp, pose);
            };
          std::vector<geometry_msgs::msg::PoseStamped> candidates;
          // Keep a still-safe waiting point as the robot approaches. Recomputing
          // its bearing every second can flip the endpoint across the obstacle.
          if (approach_anchor_ && anchor_goal_ == goal && usable(*approach_anchor_)) {
            candidates.push_back(*approach_anchor_);
          } else {
            approach_anchor_.reset();
          }
          const double toward=std::atan2(start.pose.position.y-target.y,start.pose.position.x-target.x);
          for(double radius=.4;radius<=2.001;radius+=.2) {
            for(int k=0;k<16;++k) {
              const int offset=(k+1)/2*(k%2 ? 1 : -1);
              const double angle=toward+offset*2*M_PI/16;
              auto candidate=goal;
              candidate.pose.position.x=target.x+radius*std::cos(angle);
              candidate.pose.position.y=target.y+radius*std::sin(angle);
              if (usable(candidate)) {candidates.push_back(candidate);}
            }
          }
          lock.unlock();
          const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
          unsigned attempts=0;
          for (const auto & candidate:candidates) {
            if(attempts++>=12 || std::chrono::steady_clock::now()>deadline) {break;}
            try {
              auto path=planFromStart(start,candidate,canceled);
              approach_anchor_=candidate;anchor_goal_=goal;
              RCLCPP_INFO_THROTTLE(logger_, *map_->get_clock(), 2000,
                "Occupied goal: approach (%.2f, %.2f); original target remains pending",candidate.pose.position.x,candidate.pose.position.y);
              return path;
            } catch(const nav2_core::PlannerCancelled &) {throw;}
            catch(const nav2_core::PlannerException &) {}
          }
          throw nav2_core::NoValidPathCouldBeFound("No safe approach to occupied goal");
        }
      }
    }
    approach_anchor_.reset();
    return planFromStart(start,goal,canceled);
  }
private:
  nav_msgs::msg::Path planFromStart(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> canceled)
  {
    const auto & q = start.pose.orientation;
    const Pose2 a{start.pose.position.x, start.pose.position.y,
      std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))};
    const Pose2 b{goal.pose.position.x, goal.pose.position.y, 0.0};
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
      if (footprintFree(*map, fp, a) && map->worldToMap(a.x, a.y, sx, sy) &&
        map->getCost(sx, sy) < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        lock.unlock();
        auto path = inner_->createPlan(start, goal, canceled);
        if (!path.poses.empty()) {path.poses.front() = start;}
        return path;
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
        std::vector<geometry_msgs::msg::PoseStamped> prefix;
        for (const auto & p : escape.poses) {
          auto ps = start; ps.pose.position.x = p.x; ps.pose.position.y = p.y;
          prefix.push_back(ps);
        }
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
private:
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader_;
  nav2_core::GlobalPlanner::Ptr inner_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map_;
  rclcpp::Logger logger_{rclcpp::get_logger("StartAwarePlanner")};
  double max_distance_{1.5};
  double approach_clearance_{0.20};
  std::optional<geometry_msgs::msg::PoseStamped> approach_anchor_;
  geometry_msgs::msg::PoseStamped anchor_goal_;
  bool approach_goal_{false};
};
}  // namespace nav_behaviors
PLUGINLIB_EXPORT_CLASS(nav_behaviors::StartAwarePlanner, nav2_core::GlobalPlanner)
