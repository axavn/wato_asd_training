#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Creates an empty global map where every cell is "unknown" (-1)
    void initializeMap(double resolution, double width_m, double height_m,
                       double origin_x, double origin_y, const std::string& frame_id);

    // Pastes a robot-centered costmap into the global map, given where the robot was
    // (x and y in meters, yaw in radians, all in the map frame) when the costmap was made
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x, double robot_y, double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& getMap() const { return global_map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}

#endif
