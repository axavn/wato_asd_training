#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);

    void configure(double lookahead_distance, double goal_tolerance,
                   double linear_speed, double min_linear_speed, double curvature_slowdown,
                   double max_angular_speed, double max_acceleration, double control_period,
                   double turn_enter_angle, double turn_exit_angle, double turn_gain);

    // Pure Pursuit: works out the velocity command that steers the robot along `path`.
    // Sets goal_reached to true once the robot is within goal_tolerance of the last waypoint.
    // Not const: it remembers whether the robot is turning on the spot and how fast it was going.
    geometry_msgs::msg::Twist computeVelocity(const nav_msgs::msg::Path& path,
                                              const nav_msgs::msg::Odometry& odom,
                                              bool& goal_reached);

    // Forget the remembered state (turning on the spot, current speed). Call when a path ends,
    // so the next goal starts fresh instead of in the middle of an old spin.
    void reset();

  private:
    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(const nav_msgs::msg::Path& path,
                                                                const geometry_msgs::msg::Point& robot) const;
    static double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);
    static double extractYaw(const geometry_msgs::msg::Quaternion& quat);

    rclcpp::Logger logger_;

    double lookahead_distance_ = 2.5;  // meters
    double goal_tolerance_ = 0.3;      // meters
    double linear_speed_ = 0.5;        // m/s, top speed on straights
    double min_linear_speed_ = 0.2;    // m/s, never slower than this in curves
    double curvature_slowdown_ = 1.0;  // how much curves slow the robot down
    double max_angular_speed_ = 0.3;   // rad/s
    double max_speed_step_ = 0.05;     // m/s the speed may rise per control cycle
    double turn_enter_angle_ = 1.0;    // rad; start turning on the spot above this heading error...
    double turn_exit_angle_ = 0.3;     // rad; ...and keep turning until it drops below this
    double turn_gain_ = 0.4;           // rad/s of spin per rad of heading error

    bool turning_in_place_ = false;
    double last_speed_ = 0.0;          // m/s sent last cycle, for the acceleration limit
};

}

#endif
