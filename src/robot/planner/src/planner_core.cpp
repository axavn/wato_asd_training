#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "planner_core.hpp"

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
: logger_(logger) {}

bool PlannerCore::worldToCell(const nav_msgs::msg::OccupancyGrid& map, double x, double y, CellIndex& cell) const
{
  int cell_x = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
  int cell_y = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
  if (cell_x < 0 || cell_y < 0 ||
      cell_x >= static_cast<int>(map.info.width) || cell_y >= static_cast<int>(map.info.height)) {
    return false;
  }
  cell = CellIndex(cell_x, cell_y);
  return true;
}

bool PlannerCore::nearestSafeCell(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& from,
                                  int radius_cells, int obstacle_threshold, CellIndex& result) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const CellIndex center = from;  // copy first: `from` and `result` may be the same variable
  bool found = false;
  double best_distance = std::numeric_limits<double>::infinity();

  // Check every cell in a square around the center and keep the closest safe one
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      int x = center.x + dx;
      int y = center.y + dy;
      if (x < 0 || y < 0 || x >= width || y >= height) {
        continue;
      }
      double distance = std::hypot(dx, dy);
      if (distance > radius_cells || distance >= best_distance) {
        continue;
      }
      int cost = map.data[y * width + x];
      if (cost < obstacle_threshold) {  // unknown (-1) counts as safe
        best_distance = distance;
        result = CellIndex(x, y);
        found = true;
      }
    }
  }
  return found;
}

bool PlannerCore::planPath(const nav_msgs::msg::OccupancyGrid& map,
                           double start_x, double start_y, double goal_x, double goal_y,
                           int obstacle_threshold, double cost_weight, double goal_snap_radius,
                           nav_msgs::msg::Path& path, double& target_x, double& target_y) const
{
  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  if (width == 0 || height == 0 || map.data.size() != static_cast<size_t>(width * height)) {
    return false;
  }

  CellIndex start, goal;
  if (!worldToCell(map, start_x, start_y, start)) {
    RCLCPP_WARN(logger_, "The robot is outside the map");
    return false;
  }
  if (!worldToCell(map, goal_x, goal_y, goal)) {
    RCLCPP_WARN(logger_, "The goal (%.2f, %.2f) is outside the map", goal_x, goal_y);
    return false;
  }

  // Every cell also has a single number id: y * width + x
  auto toId = [width](const CellIndex& cell) { return cell.y * width + cell.x; };

  // Unknown cells (-1) count as free: we plan optimistically, then replan as the map fills in
  auto cellCost = [&map, &toId](const CellIndex& cell) {
    int cost = map.data[toId(cell)];
    return cost < 0 ? 0 : cost;
  };

  // A goal inside (or too close to) an obstacle can't be reached safely,
  // so aim for the nearest safe cell instead
  if (cellCost(goal) >= obstacle_threshold) {
    int radius_cells = static_cast<int>(std::ceil(goal_snap_radius / map.info.resolution));
    if (!nearestSafeCell(map, goal, radius_cells, obstacle_threshold, goal)) {
      RCLCPP_WARN(logger_, "The goal is too close to an obstacle and there is no safe spot nearby");
      return false;
    }
  }
  target_x = map.info.origin.position.x + (goal.x + 0.5) * map.info.resolution;
  target_y = map.info.origin.position.y + (goal.y + 0.5) * map.info.resolution;

  // If the robot starts inside the padding around an obstacle (e.g. after being driven there by
  // hand), every neighbouring cell would count as a wall and it could never leave. So it may also
  // move through cells no costlier than the one it starts on, which only lets it back away.
  const int blocked_cost = std::max(obstacle_threshold, std::min(cellCost(start) + 1, 100));

  // h(n): straight-line distance to the goal, in cells
  auto heuristic = [&goal](const CellIndex& cell) {
    return std::hypot(cell.x - goal.x, cell.y - goal.y);
  };

  const int total_cells = width * height;
  std::vector<double> g_score(total_cells, std::numeric_limits<double>::infinity());  // g(n)
  std::vector<int> came_from(total_cells, -1);  // which cell we reached each cell from
  std::vector<bool> closed(total_cells, false);  // cells we have finished evaluating

  // The open list: cells still to evaluate, smallest f = g + h first
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  g_score[toId(start)] = 0.0;
  open.emplace(start, heuristic(start));

  // The 8 neighbouring cells: 4 straight, then 4 diagonal
  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  while (!open.empty()) {
    CellIndex current = open.top().index;
    open.pop();

    int current_id = toId(current);
    if (closed[current_id]) {
      continue;  // already evaluated through a cheaper route
    }
    closed[current_id] = true;

    if (current == goal) {
      found = true;
      break;
    }

    for (int k = 0; k < 8; ++k) {
      CellIndex next(current.x + kDx[k], current.y + kDy[k]);
      if (next.x < 0 || next.y < 0 || next.x >= width || next.y >= height) {
        continue;
      }
      int next_id = toId(next);
      if (closed[next_id]) {
        continue;
      }
      int cost = cellCost(next);
      if (cost >= blocked_cost) {
        continue;  // treat as a wall
      }

      // Moving costs the distance travelled, made more expensive near obstacles
      double step = (kDx[k] != 0 && kDy[k] != 0) ? std::sqrt(2.0) : 1.0;
      double tentative_g = g_score[current_id] + step * (1.0 + cost_weight * cost / 100.0);

      if (tentative_g < g_score[next_id]) {
        g_score[next_id] = tentative_g;
        came_from[next_id] = current_id;
        open.emplace(next, tentative_g + heuristic(next));
      }
    }
  }

  if (!found) {
    return false;
  }

  // Walk backwards from the goal to the start to recover the path
  std::vector<CellIndex> cells;
  for (int id = toId(goal); id != -1; id = came_from[id]) {
    cells.emplace_back(id % width, id / width);
  }
  std::reverse(cells.begin(), cells.end());

  // Convert cells back into points (the center of each cell, in meters)
  path.header.frame_id = map.header.frame_id;
  path.poses.clear();
  for (const auto& cell : cells) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = map.header.frame_id;
    pose.pose.position.x = map.info.origin.position.x + (cell.x + 0.5) * map.info.resolution;
    pose.pose.position.y = map.info.origin.position.y + (cell.y + 0.5) * map.info.resolution;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return true;
}

}
