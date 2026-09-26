#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Runs on a timer: checks whether the goal was reached or we ran out of time
    void timerCallback();

  private:
    // The planner's two states
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };

    void planPath();
    bool goalReached() const;
    void publishEmptyPath();  // an empty path tells the controller to stop

    robot::PlannerCore planner_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_ = State::WAITING_FOR_GOAL;

    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;  // where the user clicked
    double target_x_ = 0.0;                  // where the path actually ends: the goal, or the
    double target_y_ = 0.0;                  // nearest safe spot if the goal was too close to a wall
    geometry_msgs::msg::Pose robot_pose_;
    bool have_map_ = false;
    bool have_odom_ = false;
    rclcpp::Time goal_start_time_;

    double goal_tolerance_;
    double goal_timeout_;
    int obstacle_threshold_;
    double cost_weight_;
    double goal_snap_radius_;
};

#endif
