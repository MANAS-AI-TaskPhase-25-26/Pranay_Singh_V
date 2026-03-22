#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <queue>
#include <vector>
#include <cmath>
#include <unordered_map>
#include <algorithm>

using std::placeholders::_1;


struct GridNode {
  int x, y;
  double g, h;

  GridNode(int x_, int y_, double g_, double h_)
      : x(x_), y(y_), g(g_), h(h_) {}

  double f() const { return g + h; }
};

struct Compare {
  bool operator()(const GridNode &a, const GridNode &b) {
    return a.f() > b.f();
  }
};

class AStarPlanner : public rclcpp::Node {
public:
  AStarPlanner() : Node("a_star_planner") {

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&AStarPlanner::mapCallback, this, _1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

    RCLCPP_INFO(this->get_logger(), "A* Planner Started");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  int width_, height_;
  double resolution_;
  double origin_x_, origin_y_;
  std::vector<int8_t> grid_;

  bool isValid(int x, int y) {
    return x >= 0 && y >= 0 && x < width_ && y < height_;
  }

  bool isFree(int x, int y) {
    int val = grid_[y * width_ + x];
    return val == 0;  // free cell only
  }

  double heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);
  }

  std::vector<std::pair<int,int>> neighbors(int x, int y) {
    return {
      {x+1,y},{x-1,y},{x,y+1},{x,y-1},
      {x+1,y+1},{x-1,y-1},{x+1,y-1},{x-1,y+1}
    };
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    width_ = msg->info.width;
    height_ = msg->info.height;
    resolution_ = msg->info.resolution;

    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;

    grid_ = msg->data;

    RCLCPP_INFO(this->get_logger(), "Map received (%d x %d)", width_, height_);

    plan();
  }

  void plan() {
    int sx = 0, sy = 0;
    int gx = width_ - 1, gy = height_ - 1;

    std::priority_queue<GridNode, std::vector<GridNode>, Compare> open;
    std::unordered_map<int, std::pair<int,int>> parent;
    std::unordered_map<int, double> cost;

    auto key = [&](int x,int y){ return y*width_ + x; };

    open.emplace(sx, sy, 0.0, heuristic(sx,sy,gx,gy));
    cost[key(sx,sy)] = 0.0;

    bool found = false;

    while(!open.empty()) {
      GridNode cur = open.top();
      open.pop();

      if(cur.x == gx && cur.y == gy) {
        found = true;
        break;
      }

      for(auto &n : neighbors(cur.x, cur.y)) {
        int nx = n.first;
        int ny = n.second;

        if(!isValid(nx,ny) || !isFree(nx,ny)) continue;

        double new_cost = cost[key(cur.x,cur.y)] +
                          heuristic(cur.x,cur.y,nx,ny);

        if(!cost.count(key(nx,ny)) || new_cost < cost[key(nx,ny)]) {
          cost[key(nx,ny)] = new_cost;
          open.emplace(nx,ny,new_cost,heuristic(nx,ny,gx,gy));
          parent[key(nx,ny)] = {cur.x,cur.y};
        }
      }
    }

    if(!found) {
      RCLCPP_WARN(this->get_logger(), "No path found!");
      return;
    }

    publishPath(parent, sx, sy, gx, gy);
  }

  void publishPath(std::unordered_map<int,std::pair<int,int>> &parent,
                   int sx,int sy,int gx,int gy) {

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";

    auto key = [&](int x,int y){ return y*width_ + x; };

    int cx = gx, cy = gy;

    while(!(cx == sx && cy == sy)) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";

      
      pose.pose.position.x = origin_x_ + cx * resolution_;
      pose.pose.position.y = origin_y_ + cy * resolution_;

      path.poses.push_back(pose);

      auto p = parent[key(cx,cy)];
      cx = p.first;
      cy = p.second;
    }

    std::reverse(path.poses.begin(), path.poses.end());
    path_pub_->publish(path);

    RCLCPP_INFO(this->get_logger(), "Path published with %ld points",
                path.poses.size());
  }
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<AStarPlanner>());
  rclcpp::shutdown();
  return 0;
}
