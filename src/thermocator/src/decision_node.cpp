#include "thermocator/decision_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace thermocator {

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------

DecisionNode::DecisionNode() : Node("decision_node") {

    declare_parameter("map_frame", std::string("map"));
    declare_parameter("robot_frame", std::string("base_footprint"));
    declare_parameter("heat_detection_threshold", 20.0);
    declare_parameter("frontier_min_distance", 0.8);
    declare_parameter("scoring_radius", 1.5);
    declare_parameter("max_frontier_distance", 1.0);
    declare_parameter("w_boundary", 1.0);
    declare_parameter("w_thermal_boundary", 3.0);
    declare_parameter("w_hot_interior", 0.5);
    declare_parameter("w_cold_interior", 2.0);
    declare_parameter("revisit_penalty_radius", 0.8);
    declare_parameter("max_visited_goals", 10);
    declare_parameter("investigation_duration", 5.0);
    declare_parameter("control_rate", 1.0);

    map_frame_ = get_parameter("map_frame").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    heat_detection_threshold_ = get_parameter("heat_detection_threshold").as_double();
    frontier_min_distance_ = get_parameter("frontier_min_distance").as_double();
    scoring_radius_ = get_parameter("scoring_radius").as_double();
    max_frontier_distance_ = get_parameter("max_frontier_distance").as_double();
    w_boundary_ = get_parameter("w_boundary").as_double();
    w_thermal_boundary_ = get_parameter("w_thermal_boundary").as_double();
    w_hot_interior_ = get_parameter("w_hot_interior").as_double();
    w_cold_interior_ = get_parameter("w_cold_interior").as_double();
    revisit_penalty_radius_ = get_parameter("revisit_penalty_radius").as_double();
    max_visited_goals_ = get_parameter("max_visited_goals").as_int();
    investigation_duration_ = get_parameter("investigation_duration").as_double();
    control_rate_ = get_parameter("control_rate").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS latched_qos(1);
    latched_qos.transient_local().reliable();

    thermal_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/thermal_map", latched_qos,
        std::bind(&DecisionNode::thermalMapCallback, this, std::placeholders::_1));

    spatial_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", latched_qos,
        std::bind(&DecisionNode::spatialMapCallback, this, std::placeholders::_1));

    goal_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/thermocator/goal_markers", rclcpp::QoS(10));

    nav_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DecisionNode::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "DecisionNode ready -- "
                "heat_thresh: %.1f  scoring_radius: %.2fm  max_frontier_dist: %.2fm  "
                "weights: boundary=%.2f  thermal_boundary=%.2f  "
                "hot_interior=%.2f  cold_interior=%.2f",
                heat_detection_threshold_, scoring_radius_, max_frontier_distance_,
                w_boundary_, w_thermal_boundary_,
                w_hot_interior_, w_cold_interior_);
}

// ----------------------------------------------------------------------------
// Map callbacks
// ----------------------------------------------------------------------------

void DecisionNode::thermalMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    thermal_map_ = msg;
    thermal_map_received_ = true;
}

void DecisionNode::spatialMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(map_mutex_);
    spatial_map_ = msg;
    spatial_map_received_ = true;
}

// ----------------------------------------------------------------------------
// State machine
// ----------------------------------------------------------------------------

void DecisionNode::controlLoop() {
    switch (state_) {
    case ExplorationState::IDLE:
        handleIdle();
        break;
    case ExplorationState::SCANNING:
        handleScanning();
        break;
    case ExplorationState::NAVIGATING:
        handleNavigating();
        break;
    case ExplorationState::INVESTIGATING:
        handleInvestigating();
        break;
    case ExplorationState::COMPLETE:
        handleComplete();
        break;
    }
}

void DecisionNode::handleIdle() {
    if (!thermal_map_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for /thermal_map ...");
        return;
    }
    if (!spatial_map_received_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for /map ...");
        return;
    }
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Waiting for Nav2 action server ...");
        return;
    }
    RCLCPP_INFO(get_logger(), "All systems ready -- starting exploration");
    state_ = ExplorationState::SCANNING;
}

void DecisionNode::handleScanning() {
    const auto pose = getRobotPose();
    if (!pose.has_value()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cannot get robot pose -- skipping scan");
        return;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_copy;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        thermal_copy = thermal_map_;
        spatial_copy = spatial_map_;
    }

    // Debug map state
    int spatial_free = 0, spatial_unknown = 0, spatial_occupied = 0;
    for (const auto &v : spatial_copy->data) {
        if (v >= 0 && v <= 50)
            ++spatial_free;
        else if (v == -1)
            ++spatial_unknown;
        else
            ++spatial_occupied;
    }
    int thermal_hot = 0, thermal_cold = 0, thermal_unknown = 0;
    const auto thresh = static_cast<int8_t>(heat_detection_threshold_);
    for (const auto &v : thermal_copy->data) {
        if (v >= thresh)
            ++thermal_hot;
        else if (v == -1)
            ++thermal_unknown;
        else if (v >= 0)
            ++thermal_cold;
    }
    RCLCPP_INFO(get_logger(),
                "Map state -- spatial: free=%d unknown=%d occupied=%d | "
                "thermal: hot=%d cold=%d unknown=%d",
                spatial_free, spatial_unknown, spatial_occupied,
                thermal_hot, thermal_cold, thermal_unknown);

    std::vector<Frontier> frontiers;

    if (hasHeatData(*thermal_copy)) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Phase 2 -- thermal boundary mapping mode");
        frontiers = detectThermalFrontiers(
            *spatial_copy, *thermal_copy, pose->first, pose->second);

        if (frontiers.empty()) {
            RCLCPP_INFO(get_logger(),
                        "No thermal frontiers -- falling back to spatial exploration");
            frontiers = detectSpatialFrontiers(
                *spatial_copy, pose->first, pose->second);
        }
    } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "Phase 1 -- spatial exploration mode");
        frontiers = detectSpatialFrontiers(
            *spatial_copy, pose->first, pose->second);
    }

    if (frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "No frontiers found -- exploration complete");
        complete_start_ = std::chrono::steady_clock::now();
        state_ = ExplorationState::COMPLETE;
        return;
    }

    const auto best = selectBestFrontier(frontiers);

    RCLCPP_INFO(get_logger(),
                "Goal: (%.2f, %.2f)  score=%.3f  "
                "boundary=%.2f  thermal_bonus=%.2f  "
                "hot_interior=%.2f  cold_interior=%.2f  revisit=%.2f",
                best.world_x, best.world_y, best.final_score,
                best.boundary_score, best.thermal_bonus,
                best.hot_interior, best.cold_interior, best.revisit_penalty);

    current_goal_x_ = best.world_x;
    current_goal_y_ = best.world_y;
    goal_active_ = false;
    goal_succeeded_ = false;
    goal_failed_ = false;

    // Record visited goal
    visited_goals_.push_back({best.world_x, best.world_y});
    if (static_cast<int>(visited_goals_.size()) > max_visited_goals_)
        visited_goals_.pop_front();

    sendGoal(best.world_x, best.world_y);
    state_ = ExplorationState::NAVIGATING;
}

void DecisionNode::handleNavigating() {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
                         "NAVIGATING -- active=%d succeeded=%d failed=%d",
                         goal_active_.load(), goal_succeeded_.load(), goal_failed_.load());

    if (goal_failed_) {
        RCLCPP_WARN(get_logger(), "Goal failed -- returning to scanning");
        goal_failed_ = false;
        state_ = ExplorationState::SCANNING;
        return;
    }
    if (goal_succeeded_) {
        RCLCPP_INFO(get_logger(),
                    "Reached goal -- investigating for %.1fs", investigation_duration_);
        investigation_start_ = std::chrono::steady_clock::now();
        goal_succeeded_ = false;
        state_ = ExplorationState::INVESTIGATING;
    }
}

void DecisionNode::handleInvestigating() {
    // Steady clock avoids sim time source mismatch crash
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - investigation_start_)
                               .count();

    if (elapsed >= investigation_duration_) {
        RCLCPP_INFO(get_logger(), "Investigation done -- rescanning");
        state_ = ExplorationState::SCANNING;
    }
}

void DecisionNode::handleComplete() {
    // Only recheck every 10 seconds to avoid tight loop
    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - complete_start_)
                               .count();

    if (elapsed < 10.0) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                             "Exploration complete -- monitoring for new frontiers");
        return;
    }
    complete_start_ = std::chrono::steady_clock::now();

    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_copy;
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!spatial_map_)
            return;
        spatial_copy = spatial_map_;
    }

    const auto pose = getRobotPose();
    if (!pose.has_value())
        return;

    const auto frontiers = detectSpatialFrontiers(
        *spatial_copy, pose->first, pose->second);

    if (!frontiers.empty()) {
        RCLCPP_INFO(get_logger(), "New frontiers detected -- resuming");
        state_ = ExplorationState::SCANNING;
    }
}

// ----------------------------------------------------------------------------
// Phase detection
// ----------------------------------------------------------------------------

bool DecisionNode::hasHeatData(
    const nav_msgs::msg::OccupancyGrid &thermal) const {
    const auto thresh = static_cast<int8_t>(heat_detection_threshold_);
    for (const auto &v : thermal.data) {
        if (v >= thresh)
            return true;
    }
    return false;
}

// ----------------------------------------------------------------------------
// Phase 1 -- spatial frontiers
// Free cells (0-50) adjacent to unknown cells (-1), scored by proximity
// ----------------------------------------------------------------------------

std::vector<Frontier> DecisionNode::detectSpatialFrontiers(
    const nav_msgs::msg::OccupancyGrid &spatial,
    double robot_x, double robot_y) const {
    std::vector<Frontier> frontiers;
    const auto &info = spatial.info;
    const int nb[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

    for (uint32_t row = 1; row < info.height - 1; ++row) {
        for (uint32_t col = 1; col < info.width - 1; ++col) {

            const size_t idx = static_cast<size_t>(row) * info.width + col;

            // Cartographer free cells are in range 0-50
            if (spatial.data[idx] < 0 || spatial.data[idx] > 50)
                continue;

            bool adj_unknown = false;
            for (const auto &n : nb) {
                const size_t nidx =
                    static_cast<size_t>(row + n[0]) * info.width + (col + n[1]);
                if (spatial.data[nidx] == -1) {
                    adj_unknown = true;
                    break;
                }
            }
            if (!adj_unknown)
                continue;

            const double wx =
                info.origin.position.x + (col + 0.5) * info.resolution;
            const double wy =
                info.origin.position.y + (row + 0.5) * info.resolution;
            const double dx = wx - robot_x;
            const double dy = wy - robot_y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < frontier_min_distance_)
                continue;

            Frontier f;
            f.world_x = wx;
            f.world_y = wy;
            f.distance = dist;
            f.final_score = 1.0 / (dist + 1e-6);
            frontiers.push_back(f);
        }
    }
    return frontiers;
}

// ----------------------------------------------------------------------------
// Phase 2 -- thermal boundary frontiers
//
// Scoring strategy:
//   + boundary_score:   unknown cells sitting on the known/unknown edge
//   + thermal_bonus:    boundary cells adjacent to hot cells
//   - hot_interior:     known hot cells (mild penalty -- less interesting now)
//   - cold_interior:    known cold cells (strong penalty -- nothing new here)
//   - revisit_penalty:  proximity to recently visited goal positions
//
// Hard constraint:
//   Candidates with no known neighbor within max_frontier_distance_ are
//   discarded -- robot never ventures too far from explored space
// ----------------------------------------------------------------------------

std::vector<Frontier> DecisionNode::detectThermalFrontiers(
    const nav_msgs::msg::OccupancyGrid &spatial,
    const nav_msgs::msg::OccupancyGrid &thermal,
    double robot_x, double robot_y) const {
    std::vector<Frontier> frontiers;
    const auto &si = spatial.info;
    const auto &ti = thermal.info;
    const auto thresh = static_cast<int8_t>(heat_detection_threshold_);
    const int anb[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
    const int scan = static_cast<int>(scoring_radius_ / ti.resolution);

    for (uint32_t row = 1; row < si.height - 1; ++row) {
        for (uint32_t col = 1; col < si.width - 1; ++col) {

            const size_t sidx = static_cast<size_t>(row) * si.width + col;

            // Must be spatially free (Cartographer range 0-50)
            if (spatial.data[sidx] < 0 || spatial.data[sidx] > 50)
                continue;

            const double wx =
                si.origin.position.x + (col + 0.5) * si.resolution;
            const double wy =
                si.origin.position.y + (row + 0.5) * si.resolution;
            const double dx = wx - robot_x;
            const double dy = wy - robot_y;
            const double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < frontier_min_distance_)
                continue;

            // Locate candidate in thermal grid
            const int tc = static_cast<int>(
                (wx - ti.origin.position.x) / ti.resolution);
            const int tr = static_cast<int>(
                (wy - ti.origin.position.y) / ti.resolution);

            if (tc < 0 || tc >= static_cast<int>(ti.width) ||
                tr < 0 || tr >= static_cast<int>(ti.height))
                continue;

            double boundary_score = 0.0;
            double thermal_bonus = 0.0;
            double hot_interior = 0.0;
            double cold_interior = 0.0;
            double min_known_dist = std::numeric_limits<double>::max();

            for (int dr = -scan; dr <= scan; ++dr) {
                for (int dc = -scan; dc <= scan; ++dc) {
                    const int nr = tr + dr;
                    const int nc = tc + dc;

                    if (nr < 0 || nr >= static_cast<int>(ti.height) ||
                        nc < 0 || nc >= static_cast<int>(ti.width))
                        continue;

                    const float cell_dist =
                        std::sqrt(static_cast<float>(dr * dr + dc * dc)) *
                        ti.resolution;
                    if (cell_dist > scoring_radius_)
                        continue;

                    const size_t tidx =
                        static_cast<size_t>(nr) * ti.width + nc;
                    const int8_t tv = thermal.data[tidx];

                    if (tv == -1) {
                        // Unknown cell -- reward if it sits on the boundary
                        for (const auto &n : anb) {
                            const int ar = nr + n[0];
                            const int ac = nc + n[1];
                            if (ar < 0 || ar >= static_cast<int>(ti.height) ||
                                ac < 0 || ac >= static_cast<int>(ti.width))
                                continue;
                            const size_t aidx =
                                static_cast<size_t>(ar) * ti.width + ac;
                            const int8_t av = thermal.data[aidx];
                            if (av >= 0) {
                                // Known neighbor -- this unknown is on the boundary
                                boundary_score += 1.0 / (cell_dist + 1e-3);
                                if (av >= thresh) {
                                    // Hot known neighbor -- thermal bonus
                                    thermal_bonus +=
                                        static_cast<double>(av) / 100.0 /
                                        (cell_dist + 1e-3);
                                }
                                break;
                            }
                        }
                    } else if (tv >= thresh) {
                        // Known hot cell -- mild interior penalty
                        hot_interior += 1.0 / (cell_dist + 1e-3);
                        min_known_dist = std::min(
                            min_known_dist, static_cast<double>(cell_dist));
                    } else if (tv >= 0) {
                        // Known cold cell -- strong interior penalty
                        cold_interior += 1.0 / (cell_dist + 1e-3);
                        min_known_dist = std::min(
                            min_known_dist, static_cast<double>(cell_dist));
                    }
                }
            }

            // Hard constraint -- discard candidates too far from explored space
            if (min_known_dist > max_frontier_distance_)
                continue;

            // Revisit suppression
            double revisit_penalty = 0.0;
            for (const auto &vg : visited_goals_) {
                const double vdx = wx - vg.first;
                const double vdy = wy - vg.second;
                const double vd = std::sqrt(vdx * vdx + vdy * vdy);
                if (vd < revisit_penalty_radius_) {
                    revisit_penalty += (1.0 - vd / revisit_penalty_radius_);
                }
            }

            const double score =
                w_boundary_ * boundary_score + w_thermal_boundary_ * thermal_bonus - w_hot_interior_ * hot_interior - w_cold_interior_ * cold_interior - w_cold_interior_ * revisit_penalty;

            Frontier f;
            f.world_x = wx;
            f.world_y = wy;
            f.distance = dist;
            f.boundary_score = boundary_score;
            f.thermal_bonus = thermal_bonus;
            f.hot_interior = hot_interior;
            f.cold_interior = cold_interior;
            f.revisit_penalty = revisit_penalty;
            f.final_score = score;
            frontiers.push_back(f);
        }
    }
    return frontiers;
}

// ----------------------------------------------------------------------------
// Selection
// ----------------------------------------------------------------------------

Frontier DecisionNode::selectBestFrontier(
    std::vector<Frontier> &frontiers) const {
    return *std::max_element(
        frontiers.begin(), frontiers.end(),
        [](const Frontier &a, const Frontier &b) {
            return a.final_score < b.final_score;
        });
}

// ----------------------------------------------------------------------------
// Navigation
// ----------------------------------------------------------------------------

void DecisionNode::sendGoal(double x, double y) {
    // Pull goal slightly toward robot to avoid inflation zone boundaries
    const auto pose = getRobotPose();
    if (pose.has_value()) {
        const double dx = pose->first - x;
        const double dy = pose->second - y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 0.3) {
            x += (dx / dist) * 0.3;
            y += (dy / dist) * 0.3;
        }
    }

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = map_frame_;
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.position.z = 0.0;
    goal.pose.pose.orientation.w = 1.0;
    goal.pose.pose.orientation.x = 0.0;
    goal.pose.pose.orientation.y = 0.0;
    goal.pose.pose.orientation.z = 0.0;

    auto opts =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback =
        std::bind(&DecisionNode::goalResponseCallback, this,
                  std::placeholders::_1);
    opts.feedback_callback =
        std::bind(&DecisionNode::feedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    opts.result_callback =
        std::bind(&DecisionNode::resultCallback, this,
                  std::placeholders::_1);

    nav_client_->async_send_goal(goal, opts);
    goal_active_ = true;

    publishGoalMarker(x, y);
    RCLCPP_INFO(get_logger(), "Goal sent to (%.2f, %.2f)", x, y);
}

void DecisionNode::cancelGoal() {
    if (current_goal_handle_) {
        nav_client_->async_cancel_goal(current_goal_handle_);
        current_goal_handle_.reset();
    }
    goal_active_ = false;
}

void DecisionNode::publishGoalMarker(double x, double y) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = map_frame_;
    m.ns = "thermocator_goals";
    m.id = marker_id_++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.1;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.2;
    m.color.r = 1.0f;
    m.color.g = 0.5f;
    m.color.b = 0.0f;
    m.color.a = 1.0f;
    m.lifetime = rclcpp::Duration::from_seconds(0); // persist forever
    markers.markers.push_back(m);
    goal_marker_pub_->publish(markers);
}

// ----------------------------------------------------------------------------
// Action client callbacks
// ----------------------------------------------------------------------------

void DecisionNode::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr &handle) {
    if (!handle) {
        RCLCPP_WARN(get_logger(), "Goal rejected by Nav2");
        goal_active_ = false;
        goal_failed_ = true;
        return;
    }
    current_goal_handle_ = handle;
    RCLCPP_INFO(get_logger(), "Goal accepted by Nav2");
}

void DecisionNode::feedbackCallback(
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<
        const nav2_msgs::action::NavigateToPose::Feedback>
        feedback) {
    RCLCPP_DEBUG(get_logger(),
                 "Distance remaining: %.2fm", feedback->distance_remaining);
}

void DecisionNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    goal_active_ = false;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Navigation succeeded");
        goal_succeeded_ = true;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(get_logger(), "Navigation aborted");
        goal_failed_ = true;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(get_logger(), "Navigation cancelled");
        goal_failed_ = true;
        break;
    default:
        RCLCPP_WARN(get_logger(), "Unknown navigation result");
        goal_failed_ = true;
        break;
    }
}

// ----------------------------------------------------------------------------
// TF
// ----------------------------------------------------------------------------

std::optional<std::pair<double, double>> DecisionNode::getRobotPose() const {
    try {
        const auto t = tf_buffer_->lookupTransform(
            map_frame_, robot_frame_,
            rclcpp::Time(0),
            rclcpp::Duration::from_seconds(0.1));
        return std::make_pair(
            t.transform.translation.x,
            t.transform.translation.y);
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "TF lookup failed: %s", e.what());
        return std::nullopt;
    }
}

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
