#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

// Converts a quaternion (how ROS stores 3D rotation) into yaw: the robot's heading on the ground
static double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Read settings from config/params.yaml (the second value is the default if it is missing)
  double resolution = this->declare_parameter<double>("resolution", 0.1);
  double map_width = this->declare_parameter<double>("map_width", 40.0);
  double map_height = this->declare_parameter<double>("map_height", 40.0);
  double origin_x = this->declare_parameter<double>("origin_x", -20.0);
  double origin_y = this->declare_parameter<double>("origin_y", -20.0);
  std::string map_frame = this->declare_parameter<std::string>("map_frame", "sim_world");
  double update_period = this->declare_parameter<double>("update_period", 1.0);
  distance_threshold_ = this->declare_parameter<double>("distance_threshold", 1.5);
  yaw_threshold_ = this->declare_parameter<double>("yaw_threshold", 0.5);
  max_update_interval_ = this->declare_parameter<double>("max_update_interval", 2.0);
  max_yaw_rate_ = this->declare_parameter<double>("max_yaw_rate", 0.3);

  map_memory_.initializeMap(resolution, map_width, map_height, origin_x, origin_y, map_frame);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(update_period * 1000)),
    std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Just remember the newest costmap; the timer decides when to use it
  latest_costmap_ = *msg;
  have_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  PoseSample sample;
  sample.stamp = rclcpp::Time(msg->header.stamp).seconds();
  sample.x = msg->pose.pose.position.x;
  sample.y = msg->pose.pose.position.y;
  sample.yaw = yawFromQuaternion(msg->pose.pose.orientation);
  sample.yaw_rate = msg->twist.twist.angular.z;

  pose_history_.push_back(sample);
  if (pose_history_.size() > 50) {  // odometry arrives at ~10 Hz, so this keeps ~5 seconds
    pose_history_.pop_front();
  }
}

void MapMemoryNode::updateMap() {
  if (have_costmap_ && !pose_history_.empty() && mapUpdateDue()) {
    // Use where the robot was when the lidar scan was taken, not where it is now
    const PoseSample& pose = poseClosestTo(rclcpp::Time(latest_costmap_.header.stamp).seconds());

    // While spinning quickly, a tiny timing error would smear walls across the map, so wait
    if (std::abs(pose.yaw_rate) <= max_yaw_rate_) {
      map_memory_.integrateCostmap(latest_costmap_, pose.x, pose.y, pose.yaw);
      has_updated_ = true;
      last_update_x_ = pose.x;
      last_update_y_ = pose.y;
      last_update_yaw_ = pose.yaw;
      last_update_time_ = this->now().seconds();
    }
  }

  // Publish on every tick (including before the first update) so the planner always has a map
  nav_msgs::msg::OccupancyGrid map = map_memory_.getMap();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

bool MapMemoryNode::mapUpdateDue() const {
  if (!has_updated_) {
    return true;  // use the very first costmap right away
  }

  const PoseSample& latest = pose_history_.back();
  double moved = std::hypot(latest.x - last_update_x_, latest.y - last_update_y_);
  // remainder() wraps the angle difference into [-pi, pi], so turning past +-180 degrees counts correctly
  double turned = std::abs(std::remainder(latest.yaw - last_update_yaw_, 2.0 * M_PI));
  // The time rule matters at startup: the first scans can arrive before the simulator has loaded
  // the world, and a robot that hasn't moved yet must still get a real map
  double waited = this->now().seconds() - last_update_time_;

  return moved >= distance_threshold_ || turned >= yaw_threshold_ || waited >= max_update_interval_;
}

const MapMemoryNode::PoseSample& MapMemoryNode::poseClosestTo(double stamp) const {
  const PoseSample* best = &pose_history_.back();
  double best_gap = std::abs(best->stamp - stamp);
  for (const auto& sample : pose_history_) {
    double gap = std::abs(sample.stamp - stamp);
    if (gap < best_gap) {
      best = &sample;
      best_gap = gap;
    }
  }

  // If nothing is close in time (e.g. the clocks don't line up), fall back to the newest pose
  if (best_gap > 0.5) {
    return pose_history_.back();
  }
  return *best;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
