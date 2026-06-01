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
#include <visualization_msgs/msg/marker_array.hpp>

namespace thermocator {

struct AdvisoryGoal {
    double advisory_goal_x = 0.0;
    double advisory_goal_y = 0.0;
    rclcpp::Time advisory_stamp = rclcpp::Time(0);
    bool advisory_received = false;
};

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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr advisory_sub;

    std::shared_ptr<std::mutex> map_mutex;
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_map;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;

    std::atomic<bool> goal_active{false};
    std::atomic<bool> goal_succeeded{false};
    std::atomic<bool> goal_failed{false};

    std::chrono::steady_clock::time_point goal_sent_time;

    AdvisoryGoal advisory_goal;

    std::string map_frame;
    std::string robot_frame;
    int marker_id{0};
};

struct CoverageBox {
    double x_min = 0.0;
    double y_min = 0.0;
    double x_max = 0.0;
    double y_max = 0.0;
    bool complete = false;
};
} // namespace thermocator
