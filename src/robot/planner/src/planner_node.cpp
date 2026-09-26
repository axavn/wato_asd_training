#include <chrono>
#include <cmath>
#include <memory>

#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  // Read settings from config/params.yaml (the second value is the default if it is missing)
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.5);
  goal_timeout_ = this->declare_parameter<double>("goal_timeout", 300.0);
  obstacle_threshold_ = this->declare_parameter<int>("obstacle_threshold", 35);
  cost_weight_ = this->declare_parameter<double>("cost_weight", 5.0);
  goal_snap_radius_ = this->declare_parameter<double>("goal_snap_radius", 2.5);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  current_map_ = *msg;
  have_map_ = true;
  // The map changed, so replan in case the old path now goes through something
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  if (have_map_ && !goal_.header.frame_id.empty() && goal_.header.frame_id != current_map_.header.frame_id) {
    RCLCPP_WARN(this->get_logger(), "Goal is in frame '%s' but the map is in '%s'",
                goal_.header.frame_id.c_str(), current_map_.header.frame_id.c_str());
  }
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);

  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  goal_start_time_ = this->now();
  target_x_ = goal_.point.x;
  target_y_ = goal_.point.y;
  planPath();

  if (std::hypot(target_x_ - goal_.point.x, target_y_ - goal_.point.y) > 0.2) {
    RCLCPP_INFO(this->get_logger(), "That goal is too close to an obstacle, heading to (%.2f, %.2f) instead",
                target_x_, target_y_);
  }
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    return;
  }

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
  } else if ((this->now() - goal_start_time_).seconds() > goal_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Timed out before reaching the goal, giving up");
    state_ = State::WAITING_FOR_GOAL;
    publishEmptyPath();
  }
}

bool PlannerNode::goalReached() const {
  // Compare against where the path ends, which may have been moved away from a wall
  double dx = target_x_ - robot_pose_.position.x;
  double dy = target_y_ - robot_pose_.position.y;
  return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PlannerNode::planPath() {
  if (!have_map_ || !have_odom_) {
    RCLCPP_WARN(this->get_logger(), "Cannot plan path yet: still waiting for %s",
                !have_map_ ? "/map" : "/odom/filtered");
    return;
  }

  nav_msgs::msg::Path path;
  double target_x = 0.0;
  double target_y = 0.0;
  bool found = planner_.planPath(current_map_, robot_pose_.position.x, robot_pose_.position.y,
                                 goal_.point.x, goal_.point.y, obstacle_threshold_, cost_weight_,
                                 goal_snap_radius_, path, target_x, target_y);

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "No path found to the goal");
    publishEmptyPath();
    return;
  }

  target_x_ = target_x;
  target_y_ = target_y;
  path.header.stamp = this->now();
  path_pub_->publish(path);
}

void PlannerNode::publishEmptyPath() {
  nav_msgs::msg::Path path;
  path.header.stamp = this->now();
  path.header.frame_id = have_map_ ? current_map_.header.frame_id : "sim_world";
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
