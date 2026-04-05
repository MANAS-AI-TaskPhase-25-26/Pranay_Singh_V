#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>
#include <algorithm>

using namespace std;

struct GridNode {
  int x, y;
  double g, f;

  GridNode(int x_, int y_, double g_, double f_)
      : x(x_), y(y_), g(g_), f(f_) {}
};

struct Compare {
  bool operator()(const GridNode &a, const GridNode &b) {
    return a.f > b.f;
  }
};

class AStarPlanner : public rclcpp::Node {
public:
  AStarPlanner() : Node("a_star_planner"), map_received_(false) {

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .transient_local()
                   .reliable();

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", qos,
        std::bind(&AStarPlanner::mapCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AStarPlanner::plan, this));

    RCLCPP_INFO(this->get_logger(), "A* Planner has started");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool map_received_;

  int width_, height_;
  double resolution_, origin_x_, origin_y_;

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = msg;
    width_ = msg->info.width;
    height_ = msg->info.height;
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;

    map_received_ = true;

    RCLCPP_INFO(this->get_logger(), "Map received: %d x %d", width_, height_);
  }

  bool isValid(int x, int y) {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
  }

  bool isFree(int x, int y) {
    int idx = y * width_ + x;
    if (idx < 0 || idx >= (int)map_->data.size()) return false;
    return map_->data[idx] == 0;
  }

  double heuristic(int x1, int y1, int x2, int y2) {
    return hypot(x1 - x2, y1 - y2);
  }

  vector<pair<int, int>> neighbors(int x, int y) {
    return {
        {x + 1, y}, {x - 1, y},
        {x, y + 1}, {x, y - 1}
    };
  }

  string key(int x, int y) {
    return to_string(x) + "," + to_string(y);
  }

  void plan() {
    if (!map_received_) {
      RCLCPP_WARN(this->get_logger(), "Waiting for map");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Running A*");

    int sx = 0, sy = 0;
    int gx = width_ - 1, gy = height_ - 1;

    if (!isFree(sx, sy) || !isFree(gx, gy)) {
      RCLCPP_ERROR(this->get_logger(), "Start or Goal is occupied");
      return;
    }

    priority_queue<GridNode, vector<GridNode>, Compare> open;
    unordered_map<string, double> cost;
    unordered_map<string, pair<int, int>> parent;

    open.emplace(sx, sy, 0.0, heuristic(sx, sy, gx, gy));
    cost[key(sx, sy)] = 0.0;

    bool found = false;

    while (!open.empty()) {
      GridNode cur = open.top();
      open.pop();

      if (cur.x == gx && cur.y == gy) {
        found = true;
        break;
      }

      for (auto &n : neighbors(cur.x, cur.y)) {
        int nx = n.first;
        int ny = n.second;

        if (!isValid(nx, ny) || !isFree(nx, ny)) continue;

        double new_cost = cost[key(cur.x, cur.y)] + 1.0;

        if (!cost.count(key(nx, ny)) || new_cost < cost[key(nx, ny)]) {
          cost[key(nx, ny)] = new_cost;
          double f = new_cost + heuristic(nx, ny, gx, gy);
          open.emplace(nx, ny, new_cost, f);
          parent[key(nx, ny)] = {cur.x, cur.y};
        }
      }
    }

    if (!found) {
      RCLCPP_ERROR(this->get_logger(), "Error:No path found");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Goal reached");

    vector<pair<int, int>> path;
    int cx = gx, cy = gy;

    while (!(cx == sx && cy == sy)) {
      path.push_back({cx, cy});
      auto p = parent[key(cx, cy)];
      cx = p.first;
      cy = p.second;
    }
    path.push_back({sx, sy});
    reverse(path.begin(), path.end());

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = this->get_clock()->now();

    for (auto &p : path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = this->get_clock()->now();

      pose.pose.position.x = origin_x_ + p.first * resolution_;
      pose.pose.position.y = origin_y_ + p.second * resolution_;
      pose.pose.orientation.w = 1.0;

      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);

    RCLCPP_INFO(this->get_logger(), "Path published :(%ld points)", path.size());
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}
