#include <chrono>
#include <memory>

#include "control_node.hpp"

// How often the control loop runs, in seconds (10 Hz)
static constexpr double kControlPeriod = 0.1;

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Read settings from config/params.yaml (the second value is the default if it is missing)
  double lookahead_distance = this->declare_parameter<double>("lookahead_distance", 2.5);
  double goal_tolerance = this->declare_parameter<double>("goal_tolerance", 0.3);
  double linear_speed = this->declare_parameter<double>("linear_speed", 0.5);
  double min_linear_speed = this->declare_parameter<double>("min_linear_speed", 0.2);
  double curvature_slowdown = this->declare_parameter<double>("curvature_slowdown", 1.0);
  double max_angular_speed = this->declare_parameter<double>("max_angular_speed", 0.3);
  double max_acceleration = this->declare_parameter<double>("max_acceleration", 0.5);
  double turn_enter_angle = this->declare_parameter<double>("turn_enter_angle", 1.0);
  double turn_exit_angle = this->declare_parameter<double>("turn_exit_angle", 0.3);
  double turn_gain = this->declare_parameter<double>("turn_gain", 0.4);
  control_.configure(lookahead_distance, goal_tolerance, linear_speed, min_linear_speed,
                     curvature_slowdown, max_angular_speed, max_acceleration, kControlPeriod,
                     turn_enter_angle, turn_exit_angle, turn_gain);

  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(kControlPeriod), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  current_path_ = msg;
  if (msg->poses.empty()) {
    // The planner sends an empty path when the goal is reached (or it gave up)
    if (following_path_) {
      stopRobot();
    }
    following_path_ = false;
    control_.reset();
  } else {
    if (!following_path_) {
      control_.reset();  // a new goal: don't carry over a spin or speed from the last one
    }
    following_path_ = true;
  }
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_odom_ = msg;
}

void ControlNode::controlLoop() {
  // Skip control if we aren't following a path or have no odometry yet
  if (!following_path_ || !current_path_ || !robot_odom_) {
    return;
  }

  bool goal_reached = false;
  geometry_msgs::msg::Twist cmd_vel = control_.computeVelocity(*current_path_, *robot_odom_, goal_reached);
  cmd_vel_pub_->publish(cmd_vel);

  if (goal_reached) {
    RCLCPP_INFO(this->get_logger(), "Reached the end of the path, stopping");
    following_path_ = false;
  }
}

void ControlNode::stopRobot() {
  cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
