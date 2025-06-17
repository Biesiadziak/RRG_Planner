#include <cmath>
#include <string>
#include <memory>

#include "nav2_util/node_utils.hpp"

#include "rrg_alg/rrg_node.hpp"

namespace nav2_straightline_planner
{
bool once = true;
void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  // Configure everithing
  gen_ = std::mt19937(rd());
  marker_pub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>("random_point_marker", 10);
  graph_pub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>("graph_markers", 10);
  a_star_pub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>("a_star_path_marker", 10);

  
  id = 0;
  R = 0.5;
}
double StraightLine::heuristic(const std::tuple<float, float>& a, const std::tuple<float, float>& b)
{
  float dx = std::get<0>(a) - std::get<0>(b);
  float dy = std::get<1>(a) - std::get<1>(b);
  return std::sqrt(dx * dx + dy * dy);
}

void StraightLine::updateRadius()
{
  const std::size_t n = graph.size();
  if (n < 2) {                    
    R = std::numeric_limits<double>::infinity();
    return;
  }

  constexpr double FREE_RATIO = 0.80;              // 80 % mapy wolne od kolizji
  constexpr double ZETA_2     = M_PI;             
  const double map_area       = costmap_->getSizeInMetersX() *
                                 costmap_->getSizeInMetersY();
  const double mu_free        = map_area * FREE_RATIO;

  const double gamma_star = 2.0 * std::sqrt(1.5) *
                            std::sqrt(mu_free / ZETA_2);

  const double gamma = 0.1 * 1.1 * gamma_star;         
  const double r     = gamma * std::sqrt(std::log(static_cast<double>(n)) /
                                         static_cast<double>(n));

  R = std::min(r, eta_);    
}


void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

std::tuple<float,float> StraightLine::randomPoint()
{
  double width = costmap_->getSizeInMetersX();
  double height = costmap_->getSizeInMetersY();

  std::uniform_real_distribution<> distrib(0.0, 1.0);

  float x = (distrib(gen_) * width) / 3 - width/6;
  float y = (distrib(gen_) * height) / 3 - height/6;

  unsigned int mx, my;
  costmap_->worldToMap(x, y, mx, my);
  
  if (costmap_->getCost(mx,my) < 200)
  {
    return std::make_tuple(x, y); 
  }
  else
  {
    return randomPoint();
  }
}
std::vector<std::tuple<float, float>> StraightLine::a_star(
  std::map<std::tuple<float, float>, std::list<std::tuple<float, float>>>& graph,
  std::tuple<float, float> start,
  std::tuple<float, float> goal)
{
  std::set<std::tuple<float, float>> closed_set;
  std::map<std::tuple<float, float>, std::tuple<float, float>> came_from;
  std::map<std::tuple<float, float>, float> g_score;
  std::map<std::tuple<float, float>, float> f_score;

  auto cmp = [&f_score](const std::tuple<float, float>& a, const std::tuple<float, float>& b) {
    return f_score[a] > f_score[b];
  };

  std::priority_queue<
    std::tuple<float, float>,
    std::vector<std::tuple<float, float>>,
    decltype(cmp)> open_set(cmp);

  g_score[start] = 0.0f;
  f_score[start] = heuristic(start, goal);
  open_set.push(start);

  while (!open_set.empty())
  {
    auto current = open_set.top();
    open_set.pop();

    if (current == goal)
    {
      std::vector<std::tuple<float, float>> path;
      while (came_from.count(current))
      {
        path.push_back(current);
        current = came_from[current];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    closed_set.insert(current);

    for (const auto& neighbor : graph[current])
    {
      if (closed_set.count(neighbor)) continue;

      float tentative_g = g_score[current] + heuristic(current, neighbor);

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor])
      {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g;
        f_score[neighbor] = tentative_g + heuristic(neighbor, goal);
        open_set.push(neighbor);
      }
    }
  }

  return {};  // brak ścieżki
}


void StraightLine::find_closest(std::tuple<float, float> point)
{
  float distance = std::numeric_limits<float>::infinity();
  std::tuple<float, float> closest = std::make_tuple(
    std::numeric_limits<float>::quiet_NaN(),
    std::numeric_limits<float>::quiet_NaN()
  );

  for (const auto& [node, neighbors] : graph)
  {
    if (node != std::make_tuple(std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN()))
    {
      float point_x = std::get<0>(point);
      float point_y = std::get<1>(point);

      float node_x = std::get<0>(node);
      float node_y = std::get<1>(node);

      float dx = point_x - node_x;
      float dy = point_y - node_y;

      float dist = sqrt(dx * dx + dy * dy);
      
      if (dist < R)
      {
        bool is_vaild = check_if_valid(point, node);
        if (is_vaild) {
          graph[point].push_back(node);
          graph[node].push_back(point);       
        }
      }
      
    }
  }
}
void StraightLine::publishAStarPath(const std::vector<std::tuple<float, float>>& path_points)
{
  visualization_msgs::msg::Marker path_marker;
  path_marker.header.frame_id = "map";
  path_marker.header.stamp = node_->get_clock()->now();
  path_marker.ns = "a_star_path";
  path_marker.id = 999;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;

  path_marker.scale.x = 0.05;
  path_marker.color.r = 0.0;
  path_marker.color.g = 0.0;
  path_marker.color.b = 1.0;
  path_marker.color.a = 1.0;

  for (const auto& [x, y] : path_points)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    path_marker.points.push_back(p);
  }

  a_star_pub_->publish(path_marker);

}

bool StraightLine::check_if_valid(std::tuple<float, float> point, std::tuple<float, float> closest)
{
  float point_x = std::get<0>(point);
  float point_y = std::get<1>(point);

  float closest_x = std::get<0>(closest);
  float closest_y = std::get<1>(closest);

  int num_points = 1000;

  float dx = (point_x - closest_x) / num_points;
  float dy = (point_y - closest_y) / num_points;

  for (int i = 0; i<num_points; i++)
  {
    closest_x+=dx;
    closest_y+=dy;

    unsigned int mx, my;
    costmap_->worldToMap(closest_x, closest_y, mx, my);
    if (costmap_->getCost(mx,my) > 250)
    {
      return false;
    }
    
  }
  publishMarker(point, 0.1, 1);
  return true;
}

void StraightLine::publishMarker(std::tuple<float, float> point, float scale, int color)
{
  auto [x, y] = point;
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map"; 
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = "random_point %f", id;
  marker.id = id;
  id++;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = scale; 
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.a = 1.0; 
  if (color == 1)
  {
    marker.color.r = 1.0; 
    marker.color.g = 0.0; 
    marker.color.b = 0.0;
  }
  else if (color == 2)
  {
    marker.color.r = 0.0; 
    marker.color.g = 1.0; 
    marker.color.b = 0.0; 
  }
  else if (color == 3)
  {
    marker.color.r = 0.0; 
    marker.color.g = 0.0; 
    marker.color.b = 1.0; 
  }

  // Publish the marker
  marker_pub_->publish(marker);


  visualization_msgs::msg::Marker edge_marker;
  edge_marker.header.frame_id = "map";
  edge_marker.header.stamp = node_->get_clock()->now();
  edge_marker.ns = "edges";
  edge_marker.id = 0;
  edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  edge_marker.action = visualization_msgs::msg::Marker::ADD;
  edge_marker.scale.x = 0.01;
  edge_marker.color.r = 1.0f;
  edge_marker.color.g = 0.0f;
  edge_marker.color.b = 0.0f;
  edge_marker.color.a = 1.0;

  for (const auto& [from, neighbors] : graph) {
    float from_x = std::get<0>(from);
    float from_y = std::get<1>(from);
    if (!std::isnan(from_x) && !std::isnan(from_y))
    {
      geometry_msgs::msg::Point p1;
      p1.x = from_x;
      p1.y = from_y;
      p1.z = 0.0;

      for (const auto& to : neighbors) {
        float to_x = std::get<0>(to);
        float to_y = std::get<1>(to);
        if (!std::isnan(to_x) && !std::isnan(to_y))
        {
          geometry_msgs::msg::Point p2;
          p2.x = to_x;
          p2.y = to_y;
          p2.z = 0.0;

          edge_marker.points.push_back(p1);
          edge_marker.points.push_back(p2);
        }
      }
    }
  }
  graph_pub_->publish(edge_marker);
}


nav_msgs::msg::Path StraightLine::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> /*cancel_checker*/)
{
  nav_msgs::msg::Path global_path;
  std:: tuple<float, float> start_t = std::tuple<float, float> (start.pose.position.x, start.pose.position.y);
  std:: tuple<float, float> goal_t = std::tuple<float, float> (goal.pose.position.x, goal.pose.position.y);
  bool new_goal = first_plan || (goal_t != last_goal_);
  if (new_goal) {
    graph.clear();
    graph_built_ = false;
    path_points.clear();
    last_start_ = start_t;
    last_goal_ = goal_t;
    first_plan = false;
  }

  //path_points.clear(); 
  //graph[start_t];              
  //graph[goal_t];
  //if(graph_built_){
  //  find_closest(start_t);
  //  find_closest(goal_t);
  //  path_points = a_star(graph, start_t, goal_t);
  //}
  auto now = std::chrono::steady_clock::now();
  double seconds_since_last = std::chrono::duration<double>(now - last_update_time_).count();

  if (seconds_since_last >= update_interval_sec_) {
      RCLCPP_INFO(node_->get_logger(), "[A* update] Minęło %.2f sek., przeliczam trasę...", seconds_since_last);

      path_points.clear(); 
      graph[start_t];              
      graph[goal_t];
      if (graph_built_) {
          find_closest(start_t);
          find_closest(goal_t);
          path_points = a_star(graph, start_t, goal_t);
          publishAStarPath(path_points);
      }
      last_update_time_ = now; 
  }


  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }
  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  //WARIANT Z CIĄGŁYM DOKŁADANIEM PUNKTÓW
  if(!path_points.empty()){
    for(int n = 0; n<100; n++){
      std::tuple<float,float> point = this->randomPoint();
      find_closest(point);
      updateRadius();
      RCLCPP_INFO(node_->get_logger(), "Aktualny promień R = %.4f", R);
    }
      
  }
  
  if (!graph_built_){

    while(true){
      std::tuple<float, float> point = this->randomPoint();
      RCLCPP_INFO(this->node_->get_logger(), "Random point: %f, %f", std::get<0>(point), std::get<1>(point));
      updateRadius();
      RCLCPP_INFO(node_->get_logger(), "Aktualny promień R = %.4f", R);


      find_closest(point);

      if (heuristic(point, goal_t) < R && check_if_valid(point, goal_t)) {
        graph[point].push_back(goal_t);
        graph[goal_t].push_back(point);
      }
      if (heuristic(point, start_t) < R && check_if_valid(point, start_t)) {
        graph[point].push_back(start_t);
        graph[start_t].push_back(point);
      }
      path_points = a_star(graph, start_t, goal_t);
      if (!path_points.empty()) {
        break;
      }

    }

    graph_built_ = true;


    publishAStarPath(path_points);
  }

  for (const auto& [x, y] : path_points)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);
  RCLCPP_INFO(node_->get_logger(), "Graph has start? %s", graph.count(start_t) ? "yes" : "no");
  RCLCPP_INFO(node_->get_logger(), "Graph has goal? %s", graph.count(goal_t) ? "yes" : "no");
  RCLCPP_INFO(node_->get_logger(), "Goal has %ld connections", graph[goal_t].size());

  RCLCPP_INFO(node_->get_logger(), "A* path size: %ld", path_points.size());
  if (path_points.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "A* failed to find a path!");
    return global_path;  
  }
RCLCPP_INFO(node_->get_logger(), "A* returned path of length: %ld", path_points.size());
for (size_t i = 0; i < path_points.size(); ++i) {
  float x = std::get<0>(path_points[i]);
  float y = std::get<1>(path_points[i]);
  RCLCPP_INFO(node_->get_logger(), "  Step %lu: x=%.2f y=%.2f", i, x, y);
}


return global_path;
}
} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)