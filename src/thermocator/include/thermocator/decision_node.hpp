#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "thermocator/action_grid.hpp"
#include "thermocator/node_context.hpp"
#include "thermocator_msgs/msg/mission_state.hpp"

namespace thermocator {

struct Frontier {
    double world_x = 0.0;
    double world_y = 0.0;
    double cluster_size = 0.0;
    double corridor_gain = 0.0;
    double distance = 0.0;
};

struct ActionZone {
    double world_x = 0.0;
    double world_y = 0.0;
    double strength = 0.0;
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
    enum class State {
        SCANNING,
        NAVIGATING
    };

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

    void setGoal(double x, double y, double score);
    std::optional<std::pair<double, double>> getRobotPose() const;

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
        int max_search_cells = 200;
    };

    explicit Actor(NodeContext &ctx, const Params &p);
    bool update();
    bool isComplete() const { return complete_; }

    std::vector<ActionZone> computeZones(
        const nav_msgs::msg::OccupancyGrid &costmap,
        const nav_msgs::msg::OccupancyGrid &thermal) const;

    std::vector<ActionZone> finalizePlan(
        const std::vector<ActionZone> &merged,
        const nav_msgs::msg::OccupancyGrid &costmap,
        double rx, double ry) const;

    void setPlan(const std::vector<ActionZone> &zones);

  private:
    enum class State {
        PLANNING,
        NAVIGATING,
        ACTIONING
    };

    void handlePlanning();
    void handleNavigating();
    void handleActioning();

    std::vector<ActionZone> clusterHotSpots(
        const nav_msgs::msg::OccupancyGrid &costmap,
        const nav_msgs::msg::OccupancyGrid &thermal) const;

    std::vector<size_t> planRoute(
        const std::vector<ActionZone> &zones,
        double rx, double ry) const;

    std::pair<double, double> nudgeToFreeCell(
        double wx, double wy,
        const nav_msgs::msg::OccupancyGrid &costmap) const;

    void publishZoneMarker(const ActionZone &zone, int id);
    void publishActionMap(const ActionZone &zone);
    void setGoal(double x, double y, double score);
    std::optional<std::pair<double, double>> getRobotPose() const;

    State state_ = State::PLANNING;
    bool complete_ = false;

    bool has_plan_ = false;
    std::vector<ActionZone> pending_zones_;
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

    void updateGoalStatus();
    void publishCurrentCandidate();

    void handlePeerState();      // react to the peer's mission state
    void broadcastState();       // publish this side's mission state
    void triggerActTransition(); // become the Act coordinator
    void adoptPlan(const std::string &author,
                   const std::vector<ActionZone> &zones); // accept a peer plan
    void enterDone();                                     // stop everything when anyone finishes
    std::vector<ActionZone> currentZones();               // throttled local detection
    std::vector<ActionZone> peerZones() const;            // parse last peer state
    std::vector<ActionZone> mergeZones(
        const std::vector<ActionZone> &own,
        const std::vector<ActionZone> &peer) const;
    std::optional<std::pair<double, double>> robotPose() const;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr spatial_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Publisher<thermocator_msgs::msg::MissionState>::SharedPtr state_pub_;
    rclcpp::Subscription<thermocator_msgs::msg::MissionState>::SharedPtr peer_state_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    std::shared_ptr<std::mutex> map_mutex_;
    NodeContext ctx_;

    std::unique_ptr<Explorer> explorer_;
    std::unique_ptr<Actor> actor_;

    double goal_timeout_seconds_ = 70.0;

    enum class Phase {
        WAITING,
        PHASE1,
        PHASE2,
        DONE
    };

    Phase phase_ = Phase::WAITING;

    thermocator_msgs::msg::MissionState::SharedPtr peer_state_;
    bool plan_adopted_ = false;
    std::string plan_author_;
    std::vector<ActionZone> agreed_zones_;
    double merge_dedup_radius_ = 1.5;

    std::vector<ActionZone> cached_zones_;
    std::chrono::steady_clock::time_point last_zone_calc_{};
    bool zones_ever_calced_ = false;
};

} // namespace thermocator
