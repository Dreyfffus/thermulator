#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <rclcpp_action/client.hpp>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace thermocator {

// ----------------------------------------------------------------------------
// NodeContext
//
// Shared resources owned by DecisionNode and passed to both phases.
// Phases never own these — they only hold a reference to this struct.
// ----------------------------------------------------------------------------

struct NodeContext {
    rclcpp::Logger logger = rclcpp::get_logger("NodeContext");
    rclcpp::Clock::SharedPtr clock;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;

    using NavClient = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;
    NavClient::SharedPtr nav_client;
    NavClient::SendGoalOptions send_goal_options;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr action_map_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_marker_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zone_marker_pub;

    std::shared_ptr<std::mutex> map_mutex;
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;

    std::atomic<bool> goal_active{false};
    std::atomic<bool> goal_succeeded{false};
    std::atomic<bool> goal_failed{false};

    std::chrono::steady_clock::time_point goal_sent_time;

    std::string map_frame;
    std::string robot_frame;
    int marker_id{0};
};

// ============================================================================
// CoverageBox -- one rectangular region to sweep with boustrophedon path
// ============================================================================

struct CoverageBox {
    double x_min = 0.0;
    double y_min = 0.0;
    double x_max = 0.0;
    double y_max = 0.0;
    bool complete = false;
};
} // namespace thermocator
