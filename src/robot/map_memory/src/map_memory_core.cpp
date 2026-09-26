#include <algorithm>
#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::initializeMap(double resolution, double width_m, double height_m,
                                  double origin_x, double origin_y, const std::string& frame_id)
{
  global_map_.header.frame_id = frame_id;
  global_map_.info.resolution = resolution;
  global_map_.info.width = static_cast<uint32_t>(std::round(width_m / resolution));
  global_map_.info.height = static_cast<uint32_t>(std::round(height_m / resolution));
  global_map_.info.origin.position.x = origin_x;
  global_map_.info.origin.position.y = origin_y;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(global_map_.info.width * global_map_.info.height, -1);

  RCLCPP_INFO(logger_, "Global map: %ux%u cells in frame '%s'",
              global_map_.info.width, global_map_.info.height, frame_id.c_str());
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw)
{
  const double map_res = global_map_.info.resolution;
  const int map_w = static_cast<int>(global_map_.info.width);
  const int map_h = static_cast<int>(global_map_.info.height);
  const double map_ox = global_map_.info.origin.position.x;
  const double map_oy = global_map_.info.origin.position.y;

  const double cost_res = costmap.info.resolution;
  const int cost_w = static_cast<int>(costmap.info.width);
  const int cost_h = static_cast<int>(costmap.info.height);
  const double cost_ox = costmap.info.origin.position.x;
  const double cost_oy = costmap.info.origin.position.y;

  if (cost_w * cost_h != static_cast<int>(costmap.data.size()) || cost_res <= 0.0) {
    RCLCPP_WARN(logger_, "Received a malformed costmap, skipping it");
    return;
  }

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  // Only visit global map cells the costmap could possibly cover:
  // a square around the robot as big as the costmap's farthest corner
  const double reach = std::hypot(std::max(std::abs(cost_ox), std::abs(cost_ox + cost_w * cost_res)),
                                  std::max(std::abs(cost_oy), std::abs(cost_oy + cost_h * cost_res)));
  const int min_x = std::max(0, static_cast<int>(std::floor((robot_x - reach - map_ox) / map_res)));
  const int max_x = std::min(map_w - 1, static_cast<int>(std::ceil((robot_x + reach - map_ox) / map_res)));
  const int min_y = std::max(0, static_cast<int>(std::floor((robot_y - reach - map_oy) / map_res)));
  const int max_y = std::min(map_h - 1, static_cast<int>(std::ceil((robot_y + reach - map_oy) / map_res)));

  // We go through the GLOBAL map cells and look up each one in the costmap (rather than the
  // other way around). This way every map cell gets a value and the rotated costmap leaves no holes.
  for (int my = min_y; my <= max_y; ++my) {
    for (int mx = min_x; mx <= max_x; ++mx) {
      // Center of this global map cell, in the map frame
      double world_x = map_ox + (mx + 0.5) * map_res;
      double world_y = map_oy + (my + 0.5) * map_res;

      // The same point as seen from the robot: undo the robot's position, then its rotation
      double dx = world_x - robot_x;
      double dy = world_y - robot_y;
      double local_x =  cos_yaw * dx + sin_yaw * dy;
      double local_y = -sin_yaw * dx + cos_yaw * dy;

      // Which costmap cell is that?
      int cx = static_cast<int>(std::floor((local_x - cost_ox) / cost_res));
      int cy = static_cast<int>(std::floor((local_y - cost_oy) / cost_res));
      if (cx < 0 || cx >= cost_w || cy < 0 || cy >= cost_h) {
        continue;
      }

      int8_t new_cost = costmap.data[cy * cost_w + cx];
      if (new_cost < 0) {
        continue;  // the costmap knows nothing about this cell, keep what we remembered
      }

      // The assignment suggests letting new data overwrite old data. But our costmap marks every
      // cell without a lidar hit as free, including cells hidden behind walls, so overwriting
      // would erase walls we saw earlier. The obstacles never move, so we keep the higher cost.
      int8_t& map_cost = global_map_.data[my * map_w + mx];
      map_cost = std::max(map_cost, new_cost);
    }
  }
}

}
