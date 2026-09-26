#include <chrono>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Read settings from config/params.yaml (the second value is the default if it is missing)
  double resolution = this->declare_parameter<double>("resolution", 0.1);
  int width = this->declare_parameter<int>("width", 300);
  int height = this->declare_parameter<int>("height", 300);
  double inflation_radius = this->declare_parameter<double>("inflation_radius", 1.8);
  int max_inflation_cost = this->declare_parameter<int>("max_inflation_cost", 90);
  costmap_.configure(resolution, width, height, inflation_radius, max_inflation_cost);

  // SensorDataQoS accepts the lidar's messages whether they are sent reliably or best-effort
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(),
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  costmap_pub_->publish(costmap_.buildCostmap(*scan));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
