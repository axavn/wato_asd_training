#include <algorithm>
#include <cmath>
#include <limits>

#include "control_core.hpp"

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void ControlCore::configure(double lookahead_distance, double goal_tolerance,
                            double linear_speed, double min_linear_speed, double curvature_slowdown,
                            double max_angular_speed, double max_acceleration, double control_period,
                            double turn_enter_angle, double turn_exit_angle, double turn_gain)
{
  lookahead_distance_ = lookahead_distance;
  goal_tolerance_ = goal_tolerance;
  linear_speed_ = linear_speed;
  min_linear_speed_ = min_linear_speed;
  curvature_slowdown_ = curvature_slowdown;
  max_angular_speed_ = max_angular_speed;
  max_speed_step_ = max_acceleration * control_period;
  turn_enter_angle_ = turn_enter_angle;
  turn_exit_angle_ = turn_exit_angle;
  turn_gain_ = turn_gain;
}

void ControlCore::reset()
{
  turning_in_place_ = false;
  last_speed_ = 0.0;
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(const nav_msgs::msg::Path& path,
                                                       const nav_msgs::msg::Odometry& odom,
                                                       bool& goal_reached)
{
  geometry_msgs::msg::Twist cmd_vel;  // starts as all zeros, which means "stop"
  goal_reached = false;

  if (path.poses.empty()) {
    reset();
    return cmd_vel;
  }

  const geometry_msgs::msg::Point& robot = odom.pose.pose.position;
  double yaw = extractYaw(odom.pose.pose.orientation);

  // Close enough to the end of the path? Then stop.
  double distance_to_goal = computeDistance(robot, path.poses.back().pose.position);
  if (distance_to_goal < goal_tolerance_) {
    goal_reached = true;
    reset();
    return cmd_vel;
  }

  auto target = findLookaheadPoint(path, robot);
  if (!target) {
    return cmd_vel;
  }

  // Express the lookahead point in the robot's own frame (x = straight ahead, y = to the left)
  double dx = target->x - robot.x;
  double dy = target->y - robot.y;
  double local_x =  std::cos(yaw) * dx + std::sin(yaw) * dy;
  double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  double distance_sq = local_x * local_x + local_y * local_y;
  if (distance_sq < 1e-6) {
    return cmd_vel;
  }

  // Angle between where the robot is facing and where the lookahead point is
  double heading_error = std::atan2(local_y, local_x);

  // Turning on the spot uses two thresholds so the robot doesn't flip back and forth between
  // spinning and driving: start above turn_enter_angle, stop only once below turn_exit_angle
  double turn_threshold = turning_in_place_ ? turn_exit_angle_ : turn_enter_angle_;
  turning_in_place_ = std::abs(heading_error) > turn_threshold;
  if (turning_in_place_) {
    // Spin faster when far off, slower as we line up. Kept gentle on purpose: when the simulator
    // runs slowly, commands can reach the robot seconds late, and a fast spin then overshoots
    // over and over instead of lining up.
    cmd_vel.angular.z = std::clamp(turn_gain_ * heading_error, -max_angular_speed_, max_angular_speed_);
    last_speed_ = 0.0;  // standing still, so driving off afterwards starts from zero speed
    return cmd_vel;
  }

  // Pure Pursuit: curvature of the circular arc from the robot to the lookahead point
  double curvature = 2.0 * local_y / distance_sq;

  // Full speed on straights, slower in curves, and slower again for the last stretch to the goal
  double speed = linear_speed_ / (1.0 + curvature_slowdown_ * std::abs(curvature));
  speed = std::max(speed, min_linear_speed_);
  speed = std::min(speed, std::max(0.1, distance_to_goal));

  // If that would still turn faster than max_angular_speed, slow down more instead of capping the
  // turn rate: capping it would make the robot swing wider than the arc it is trying to follow
  if (std::abs(speed * curvature) > max_angular_speed_) {
    speed = max_angular_speed_ / std::abs(curvature);
  }

  // Speed up gradually (braking is instant)
  speed = std::min(speed, last_speed_ + max_speed_step_);
  last_speed_ = speed;

  cmd_vel.linear.x = speed;
  cmd_vel.angular.z = speed * curvature;
  return cmd_vel;
}

std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path, const geometry_msgs::msg::Point& robot) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  // 1) Find the waypoint closest to the robot, so we never aim at points we already passed
  size_t closest = 0;
  double closest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double distance = computeDistance(robot, path.poses[i].pose.position);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest = i;
    }
  }

  // 2) From there, walk forward to the first waypoint at least lookahead_distance away
  for (size_t i = closest; i < path.poses.size(); ++i) {
    if (computeDistance(robot, path.poses[i].pose.position) >= lookahead_distance_) {
      return path.poses[i].pose.position;
    }
  }

  // 3) We are near the end of the path: aim straight at the last point
  return path.poses.back().pose.position;
}

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat)
{
  // Standard quaternion -> yaw (rotation around the vertical axis) formula
  return std::atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                    1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z));
}

}
