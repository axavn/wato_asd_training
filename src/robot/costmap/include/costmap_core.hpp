#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Sets the grid size and inflation settings (called once by the node on startup)
    void configure(double resolution, int width, int height,
                   double inflation_radius, int max_inflation_cost);

    // Converts one laser scan into a costmap centered on the robot
    nav_msgs::msg::OccupancyGrid buildCostmap(const sensor_msgs::msg::LaserScan& scan);

  private:
    // Converts a point in the robot's frame (meters) into a grid cell. Returns false if it is off the grid.
    bool pointToCell(double x, double y, int& cell_x, int& cell_y) const;
    void markObstacle(int cell_x, int cell_y);
    void inflateObstacles();

    rclcpp::Logger logger_;

    double resolution_ = 0.1;     // meters per cell
    int width_ = 300;             // cells
    int height_ = 300;            // cells
    double inflation_radius_ = 1.8;
    int max_inflation_cost_ = 90;

    // The grid is stored as one long row-major array: index = y * width_ + x
    std::vector<int8_t> grid_;
    std::vector<std::pair<int, int>> obstacle_cells_;
};

}

#endif
