#ifndef RRG_ALG__RRG_NODE_HPP_
#define RRG_ALG__RRG_NODE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "visualization_msgs/msg/marker.hpp"

namespace nav2_straightline_planner
{

class StraightLine : public nav2_core::GlobalPlanner
{
public:
  StraightLine() = default;
  ~StraightLine() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;
  
  
  // My variables
  std::random_device rd;
  std::mt19937 gen_;
  int id;
  double eta_{5.0};
  double update_interval_sec_ = 3.0;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr a_star_pub_;

  std::map<std::tuple<float, float>, std::list<std::tuple<float, float> >> graph;
  float R;
  std::tuple<float, float> last_goal_;
  std::tuple<float, float> last_start_;
  bool first_plan = true;


private:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;

  // The global frame of the costmap
  std::string global_frame_, name_;

  double interpolation_resolution_;
  

  // My functions
  bool graph_built_ = false;
  std::vector<std::tuple<float, float>> path_points;
  std::tuple<float, float> randomPoint();
  void updateRadius();
  void publishMarker(std::tuple<float, float> point, float scale, int color);
  void find_closest(std::tuple<float, float> point);
  bool check_if_valid(std::tuple<float, float> point, std::tuple<float, float> closest);
  std::chrono::steady_clock::time_point last_update_time_{std::chrono::steady_clock::now()};
  

  std::vector<std::tuple<float, float>> a_star(
    std::map<std::tuple<float, float>, std::list<std::tuple<float, float>>>& graph,
    std::tuple<float, float> start,
    std::tuple<float, float> goal);

  void publishAStarPath(const std::vector<std::tuple<float, float>>& path_points);

  double heuristic(const std::tuple<float, float>& a, const std::tuple<float, float>& b);
  
};

}  // namespace nav2_straightline_planner

#endif  // NAV2_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_