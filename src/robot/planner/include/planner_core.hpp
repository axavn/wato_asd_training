#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace robot
{

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex
{
  int x;
  int y;

  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}

  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }

  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h

  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b) const
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);

    // Uses A* to plan a path on `map` from (start_x, start_y) to (goal_x, goal_y), in meters.
    // Cells with a cost >= obstacle_threshold are treated as walls. cost_weight controls how
    // strongly the path avoids high-cost cells near obstacles. If the goal is too close to an
    // obstacle, it is moved to the nearest safe cell within goal_snap_radius meters.
    // (target_x, target_y) is set to where the path actually ends.
    // Returns false if there is no path.
    bool planPath(const nav_msgs::msg::OccupancyGrid& map,
                  double start_x, double start_y, double goal_x, double goal_y,
                  int obstacle_threshold, double cost_weight, double goal_snap_radius,
                  nav_msgs::msg::Path& path, double& target_x, double& target_y) const;

  private:
    // Converts a point in meters into a map cell. Returns false if it is outside the map.
    bool worldToCell(const nav_msgs::msg::OccupancyGrid& map, double x, double y, CellIndex& cell) const;

    // Finds the closest cell to `from` (within radius_cells) whose cost is below obstacle_threshold
    bool nearestSafeCell(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& from,
                         int radius_cells, int obstacle_threshold, CellIndex& result) const;

    rclcpp::Logger logger_;
};

}

#endif
