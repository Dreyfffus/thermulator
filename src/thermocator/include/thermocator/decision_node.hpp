#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <random>
#include <string>
#include <vector>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "thermocator/action_grid.hpp"
#include "thermocator/node_context.hpp"

namespace thermocator {

struct Frontier {
    double world_x = 0.0;
    double world_y = 0.0;
    double cluster_size = 0.0;
    double corridor_gain = 0.0;
    double distance = 0.0;
};

class Explorer {
  public:
    struct Params {
        double sensor_coverage_radius = 0.3;
        double goal_timeout_seconds = 30.0;
        double goal_min_distance = 0.5;
        double coverage_threshold = 0.95;
        double rescan_interval_seconds = 8.0;
        double radius_initial = 1.5;
        double radius_step = 0.5;
        double radius_max = 8.0;
        int samples_per_cycle = 40;
        double corridor_bonus = 0.3;
    };

    explicit Explorer(NodeContext &ctx, const Params &p);

    bool update();
    bool isComplete() const { return complete_; }

  private:
    enum class State { SCANNING,
                       NAVIGATING };

    void handleScanning();
    void handleNavigating();

    double computeCoverageRatio(
        const nav_msgs::msg::OccupancyGrid &spatial,
        const nav_msgs::msg::OccupancyGrid &thermal) const;

    std::vector<Frontier> detectTargets(
        const nav_msgs::msg::OccupancyGrid &thermal,
        const nav_msgs::msg::OccupancyGrid &costmap,
        double rx, double ry) const;

    Frontier chooseTarget(
        std::vector<Frontier> &candidates,
        const nav_msgs::msg::OccupancyGrid &costmap,
        const nav_msgs::msg::OccupancyGrid &thermal,
        double rx, double ry) const;

    double estimateGain(
        const nav_msgs::msg::OccupancyGrid &costmap,
        const nav_msgs::msg::OccupancyGrid &thermal,
        double rx, double ry, double gx, double gy) const;

    void sendGoal(double x, double y);
    void checkTimeout();
    std::optional<std::pair<double, double>> getRobotPose() const;
    void publishGoalMarker();

    State state_ = State::SCANNING;
    bool complete_ = false;
    double _current_goal_x = 0.0;
    double _current_goal_y = 0.0;

    mutable double sample_radius_;
    mutable std::mt19937 rng_;

    NodeContext &ctx_;
    Params default_params_;
};

class Actor {
  public:
    struct Params {
        double heat_threshold = 60.0;
        double cluster_radius = 1.5;
        double base_sigma = 0.4;
        double action_delay = 1.0;
    };

    explicit Actor(NodeContext &ctx, const Params &p);
    bool update();
    bool isComplete() const { return complete_; }

  private:
    enum class State { PLANNING,
                       NAVIGATING,
                       ACTIONING };

    struct ActionZone {
        double world_x = 0.0;
        double world_y = 0.0;
        double strength = 0.0;
    };

    void handlePlanning();
    void handleNavigating();
    void handleActioning();

    std::vector<ActionZone> clusterHotSpots(
        const nav_msgs::msg::OccupancyGrid &thermal) const;

    std::vector<size_t> planRoute(
        const std::vector<ActionZone> &zones,
        double rx, double ry) const;

    void publishZoneMarker(const ActionZone &zone, int id);
    void publishActionMap(const ActionZone &zone);
    void sendGoal(double x, double y);
    std::optional<std::pair<double, double>> getRobotPose() const;

    State state_ = State::PLANNING;
    bool complete_ = false;

    std::vector<ActionZone> zones_;
    std::vector<size_t> route_;
    size_t current_idx_ = 0;

    ActionGrid action_grid_;

    std::chrono::steady_clock::time_point action_start_;

    NodeContext &ctx_;
    Params params_;
};

class DecisionNode : public rclcpp::Node {
  public:
    explicit DecisionNode();
    ~DecisionNode() override = default;

  private:
    void thermalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void spatialMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void controlLoop();

    void goalResponseCallback(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::SharedPtr &handle);
    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<
            const nav2_msgs::action::NavigateToPose::Feedback>
            fb);
    void resultCallback(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr spatial_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<std::mutex> map_mutex_;
    NodeContext ctx_;

    std::unique_ptr<Explorer> explorer_;
    std::unique_ptr<Actor> actor_;

    enum class Phase { WAITING,
                       PHASE1,
                       PHASE2,
                       DONE };
    Phase phase_ = Phase::WAITING;
};

} // namespace thermocator
