#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
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

namespace thermocator {

enum class ExplorationState {
    IDLE,
    SCANNING,
    NAVIGATING,
    INVESTIGATING,
    COMPLETE
};

struct Frontier {
    double world_x = 0.0;
    double world_y = 0.0;
    double distance = 0.0;
    double boundary_score = 0.0;
    double thermal_bonus = 0.0;
    double hot_interior = 0.0;
    double cold_interior = 0.0;
    double revisit_penalty = 0.0;
    double final_score = 0.0;
};

class DecisionNode : public rclcpp::Node {
  public:
    explicit DecisionNode();
    ~DecisionNode() override = default;

  private:
    // Callbacks
    void thermalMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void spatialMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    // Control loop
    void controlLoop();
    void handleIdle();
    void handleScanning();
    void handleNavigating();
    void handleInvestigating();
    void handleComplete();

    // Phase detection
    bool hasHeatData(const nav_msgs::msg::OccupancyGrid &thermal) const;

    // Frontier detection
    std::vector<Frontier> detectSpatialFrontiers(
        const nav_msgs::msg::OccupancyGrid &spatial,
        double robot_x, double robot_y) const;

    std::vector<Frontier> detectThermalFrontiers(
        const nav_msgs::msg::OccupancyGrid &spatial,
        const nav_msgs::msg::OccupancyGrid &thermal,
        double robot_x, double robot_y) const;

    Frontier selectBestFrontier(std::vector<Frontier> &frontiers) const;

    // Navigation
    void sendGoal(double x, double y);
    void cancelGoal();
    void publishGoalMarker(double x, double y);

    // Action client callbacks
    void goalResponseCallback(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::SharedPtr &handle);

    void feedbackCallback(
        rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<
            const nav2_msgs::action::NavigateToPose::Feedback>
            feedback);

    void resultCallback(
        const rclcpp_action::ClientGoalHandle<
            nav2_msgs::action::NavigateToPose>::WrappedResult &result);

    // TF
    std::optional<std::pair<double, double>> getRobotPose() const;

    // -------------------------------------------------------------------------
    // Subscriptions / publishers
    // -------------------------------------------------------------------------
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr spatial_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr goal_marker_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // -------------------------------------------------------------------------
    // Maps
    // -------------------------------------------------------------------------
    std::mutex map_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_map_;
    bool thermal_map_received_ = false;
    bool spatial_map_received_ = false;

    // -------------------------------------------------------------------------
    // State
    // -------------------------------------------------------------------------
    ExplorationState state_ = ExplorationState::IDLE;
    std::atomic<bool> goal_active_{false};
    std::atomic<bool> goal_succeeded_{false};
    std::atomic<bool> goal_failed_{false};
    double current_goal_x_ = 0.0;
    double current_goal_y_ = 0.0;

    // Investigation timer -- uses steady clock to avoid sim time source mismatch
    std::chrono::steady_clock::time_point investigation_start_;

    // Complete state recheck timer
    std::chrono::steady_clock::time_point complete_start_;

    // Visited goal memory for revisit suppression
    std::deque<std::pair<double, double>> visited_goals_;

    // Goal marker ID counter
    int marker_id_ = 0;

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------

    // Frames
    std::string map_frame_;
    std::string robot_frame_;

    // Phase detection
    double heat_detection_threshold_; // occupancy value (0-100) = "hot"

    // Spatial frontier detection
    double frontier_min_distance_; // min distance from robot to consider a frontier

    // Thermal frontier scoring
    double scoring_radius_;        // meters around candidate to evaluate
    double max_frontier_distance_; // hard cutoff: discard candidates with no
                                   // known neighbor within this distance (meters)

    // Scoring weights
    double w_boundary_;         // reward: unknown cells on known/unknown boundary
    double w_thermal_boundary_; // bonus:  boundary cells adjacent to hot cells
    double w_hot_interior_;     // penalty: known hot cells inside scoring radius
    double w_cold_interior_;    // penalty: known cold cells inside scoring radius
                                //          (should be > w_hot_interior)

    // Revisit suppression
    double revisit_penalty_radius_; // meters: goals within this radius are penalised
    int max_visited_goals_;         // how many past goals to remember

    // Navigation timing
    double investigation_duration_; // seconds to wait at goal before rescanning
    double control_rate_;           // Hz
};

} // namespace thermocator
