#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

static constexpr int8_t kFreeCost = 0;
static constexpr int8_t kObstacleCost = 100;

CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {}

void CostmapCore::configure(double resolution, int width, int height,
                            double inflation_radius, int max_inflation_cost)
{
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  inflation_radius_ = inflation_radius;
  max_inflation_cost_ = max_inflation_cost;
  grid_.assign(width_ * height_, kFreeCost);

  RCLCPP_INFO(logger_, "Costmap: %dx%d cells at %.2f m/cell, inflation radius %.2f m",
              width_, height_, resolution_, inflation_radius_);
}

nav_msgs::msg::OccupancyGrid CostmapCore::buildCostmap(const sensor_msgs::msg::LaserScan& scan)
{
  // Step 1: Start from an empty (all free) grid
  std::fill(grid_.begin(), grid_.end(), kFreeCost);
  obstacle_cells_.clear();

  // Step 2: Convert every laser reading into a grid cell and mark it as an obstacle
  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    double range = scan.ranges[i];
    // Skip beams that hit nothing (inf) or fall outside the sensor's valid range
    if (!std::isfinite(range) || range < scan.range_min || range >= scan.range_max) {
      continue;
    }

    // Polar (angle, distance) -> Cartesian (x ahead of the robot, y to its left)
    double angle = scan.angle_min + i * scan.angle_increment;
    double x = range * std::cos(angle);
    double y = range * std::sin(angle);

    int cell_x, cell_y;
    if (pointToCell(x, y, cell_x, cell_y)) {
      markObstacle(cell_x, cell_y);
    }
  }

  // Step 3: Spread cost around each obstacle so the robot keeps its distance
  inflateObstacles();

  // Step 4: Package the grid into an OccupancyGrid message
  nav_msgs::msg::OccupancyGrid costmap;
  costmap.header = scan.header;  // same timestamp and frame as the lidar scan
  costmap.info.resolution = resolution_;
  costmap.info.width = width_;
  costmap.info.height = height_;
  // The origin is the bottom-left corner of the grid. This puts the robot in the middle.
  costmap.info.origin.position.x = -width_ * resolution_ / 2.0;
  costmap.info.origin.position.y = -height_ * resolution_ / 2.0;
  costmap.info.origin.orientation.w = 1.0;
  costmap.data = grid_;
  return costmap;
}

bool CostmapCore::pointToCell(double x, double y, int& cell_x, int& cell_y) const
{
  double origin_x = -width_ * resolution_ / 2.0;
  double origin_y = -height_ * resolution_ / 2.0;
  cell_x = static_cast<int>(std::floor((x - origin_x) / resolution_));
  cell_y = static_cast<int>(std::floor((y - origin_y) / resolution_));
  return cell_x >= 0 && cell_x < width_ && cell_y >= 0 && cell_y < height_;
}

void CostmapCore::markObstacle(int cell_x, int cell_y)
{
  grid_[cell_y * width_ + cell_x] = kObstacleCost;
  obstacle_cells_.emplace_back(cell_x, cell_y);
}

void CostmapCore::inflateObstacles()
{
  int radius_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (const auto& [obstacle_x, obstacle_y] : obstacle_cells_) {
    // Look at every cell in a square around the obstacle...
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        int x = obstacle_x + dx;
        int y = obstacle_y + dy;
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
          continue;
        }

        // ...but only the ones inside the inflation circle
        double distance = std::hypot(dx, dy) * resolution_;
        if (distance == 0.0 || distance > inflation_radius_) {
          continue;
        }

        // cost = max_cost * (1 - distance / inflation_radius): high near the obstacle, 0 at the edge
        int cost = static_cast<int>(max_inflation_cost_ * (1.0 - distance / inflation_radius_));

        // Only raise a cell's cost, never lower it (another obstacle may be closer)
        int8_t& cell = grid_[y * width_ + x];
        if (cost > cell) {
          cell = static_cast<int8_t>(cost);
        }
      }
    }
  }
}

}
