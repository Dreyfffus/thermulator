#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/client.hpp>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "thermocator_msgs/msg/goal_candidate.hpp"

namespace thermocator
{

struct NodeContext
{
  rclcpp::Logger logger = rclcpp::get_logger("NodeContext");
  rclcpp::Clock::SharedPtr clock;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;

    // Decision nodes no longer drive Nav2 directly. They publish scored goal
    // candidates to the arbiter, which selects the best one and sends it to Nav2.
  rclcpp::Publisher<thermocator_msgs::msg::GoalCandidate>::SharedPtr goal_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr action_map_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_marker_pub;

  std::shared_ptr<std::mutex> map_mutex;
  nav_msgs::msg::OccupancyGrid::SharedPtr thermal_map;
  nav_msgs::msg::OccupancyGrid::SharedPtr spatial_map;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;

    // Goal lifecycle is tracked locally (robot-pose proximity + timeout) because
    // this node receives no Nav2 action feedback anymore.
  std::atomic<bool> goal_active{false};
  std::atomic<bool> goal_succeeded{false};
  std::atomic<bool> goal_failed{false};
  std::chrono::steady_clock::time_point goal_sent_time;

  double current_goal_x = 0.0;
  double current_goal_y = 0.0;
  double current_score = 0.0;
  double arrival_radius = 0.4;

  std::string map_frame;
  std::string robot_frame;
  std::string goal_source = "LOCAL";   // "LOCAL" (D38) or "TWINNED" (D1)
  int marker_id{0};
};

struct CoverageBox
{
  double x_min = 0.0;
  double y_min = 0.0;
  double x_max = 0.0;
  double y_max = 0.0;
  bool complete = false;
};
} // namespace thermocator
