#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Runs on a timer: merges the latest costmap into the map (when needed) and publishes the map
    void updateMap();

  private:
    // Where the robot was at one moment in time
    struct PoseSample {
      double stamp;     // seconds
      double x;
      double y;
      double yaw;
      double yaw_rate;  // how fast the robot is turning (rad/s)
    };

    // Returns the recorded robot pose whose timestamp is closest to `stamp`
    const PoseSample& poseClosestTo(double stamp) const;

    // True when the map should take in a new costmap: the first time, or once the robot has
    // moved far enough, turned far enough, or enough time has passed since the last update
    bool mapUpdateDue() const;

    robot::MapMemoryCore map_memory_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::OccupancyGrid latest_costmap_;
    bool have_costmap_ = false;

    // The last few seconds of robot poses, so each costmap can be matched with where the robot was
    std::deque<PoseSample> pose_history_;

    // Where the robot was, and when, at the last map update
    bool has_updated_ = false;
    double last_update_x_ = 0.0;
    double last_update_y_ = 0.0;
    double last_update_yaw_ = 0.0;
    double last_update_time_ = 0.0;  // seconds

    double distance_threshold_;
    double yaw_threshold_;
    double max_update_interval_;
    double max_yaw_rate_;
};

#endif
