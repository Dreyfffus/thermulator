#include "thermocator/decision_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <rclcpp/logging.hpp>

namespace thermocator {

Explorer::Explorer(NodeContext &ctx, const Params &p)
    : sample_radius_(p.radius_initial), rng_(std::random_device{}()), ctx_(ctx), default_params_(p) {}

bool Explorer::update() {
    if (complete_)
        return true;
    switch (state_) {
    case State::SCANNING:
        handleScanning();
        break;
    case State::NAVIGATING:
        handleNavigating();
        break;
    }
    return complete_;
}

void Explorer::handleScanning() {
    const auto pose = getRobotPose();
    if (!pose.has_value())
        return;

    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_copy;
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_copy;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_copy;
    {
        std::lock_guard<std::mutex> lk(*ctx_.map_mutex);
        costmap_copy = ctx_.costmap;
        thermal_copy = ctx_.thermal_map;
        spatial_copy = ctx_.spatial_map;
    }
    if (!costmap_copy || !thermal_copy || !spatial_copy)
        return;

    const double cov = computeCoverageRatio(*spatial_copy, *thermal_copy);
    RCLCPP_INFO_THROTTLE(ctx_.logger, *ctx_.clock, 3000,
                         "[Explorer] Coverage %.1f%%  sample_r=%.1fm",
                         cov * 100.0, sample_radius_);

    if (cov >= default_params_.coverage_threshold) {
        RCLCPP_INFO(ctx_.logger, "[Explorer] Coverage complete");
        complete_ = true;
        return;
    }

    auto ft = detectTargets(
        *thermal_copy, *costmap_copy,
        pose->first, pose->second);

    if (!ft.empty()) {
        const auto best = chooseTarget(
            ft, *costmap_copy, *thermal_copy, pose->first, pose->second);

        RCLCPP_INFO(ctx_.logger,
                    "[Explorer] Frontier (%.2f,%.2f) "
                    "cg=%.0f d=%.2f r=%.1f",
                    best.world_x, best.world_y,
                    best.corridor_gain, best.distance, sample_radius_);

        ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;
        _current_goal_x = best.world_x;
        _current_goal_y = best.world_y;
        publishGoalMarker();
        sendGoal(best.world_x, best.world_y);
        state_ = State::NAVIGATING;
        return;
    }

    sample_radius_ = std::min(
        sample_radius_ + default_params_.radius_step,
        default_params_.radius_max);

    RCLCPP_INFO_THROTTLE(ctx_.logger, *ctx_.clock, 2000,
                         "[Explorer] No targets -- radius -> %.1fm",
                         sample_radius_);

    if (sample_radius_ >= default_params_.radius_max) {
        RCLCPP_INFO(ctx_.logger,
                    "[Explorer] Radius maxed -- coverage %.1f%% -- done",
                    cov * 100.0);
        complete_ = true;
    }
}

void Explorer::handleNavigating() {
    checkTimeout();

    if (ctx_.goal_failed) {
        ctx_.goal_failed = false;
        RCLCPP_WARN(ctx_.logger, "[Explorer] Goal failed (%.2f,%.2f)",
                    _current_goal_x, _current_goal_y);
        ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;
        state_ = State::SCANNING;
        return;
    }

    if (ctx_.goal_succeeded) {
        ctx_.goal_succeeded = false;
        ctx_.goal_active = false;
        state_ = State::SCANNING;
        return;
    }

    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - ctx_.goal_sent_time)
                               .count();
    if (elapsed > default_params_.rescan_interval_seconds) {
        RCLCPP_INFO(ctx_.logger, "[Explorer] Rescan interval -- re-evaluating");
        ctx_.nav_client->async_cancel_all_goals();
        ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;
        state_ = State::SCANNING;
    }
}

std::vector<Frontier> Explorer::detectTargets(
    const nav_msgs::msg::OccupancyGrid &thermal,
    const nav_msgs::msg::OccupancyGrid &costmap,
    double rx, double ry) const {
    const auto &ci = costmap.info;
    const auto &ti = thermal.info;

    std::uniform_real_distribution<double> uniform(-sample_radius_, sample_radius_);

    std::vector<Frontier> valid;
    valid.reserve(default_params_.samples_per_cycle);

    const int max_attempts = default_params_.samples_per_cycle * 4;

    for (int attempt = 0; attempt < max_attempts && (int)valid.size() < default_params_.samples_per_cycle; ++attempt) {

        const double ox = uniform(rng_);
        const double oy = uniform(rng_);
        const double d2 = ox * ox + oy * oy;
        if (d2 > sample_radius_ * sample_radius_)
            continue;

        const double dist = std::sqrt(d2);
        if (dist < default_params_.goal_min_distance)
            continue;

        const double wx = rx + ox;
        const double wy = ry + oy;

        const int cc = static_cast<int>((wx - ci.origin.position.x) / ci.resolution);
        const int cr = static_cast<int>((wy - ci.origin.position.y) / ci.resolution);
        if (cc < 0 || cc >= (int)ci.width || cr < 0 || cr >= (int)ci.height)
            continue;
        const auto cv = costmap.data[static_cast<size_t>(cr) * ci.width + cc];
        if (cv < 0 || cv >= 99)
            continue;

        const int tc = static_cast<int>((wx - ti.origin.position.x) / ti.resolution);
        const int tr = static_cast<int>((wy - ti.origin.position.y) / ti.resolution);
        if (tc < 0 || tc >= (int)ti.width || tr < 0 || tr >= (int)ti.height) {
            valid.push_back({wx, wy, 0.0, 0.0, dist});
            continue;
        }
        if (thermal.data[static_cast<size_t>(tr) * ti.width + tc] == -1)
            valid.push_back({wx, wy, 0.0, 0.0, dist});
    }

    return valid;
}

Frontier Explorer::chooseTarget(
    std::vector<Frontier> &candidates,
    const nav_msgs::msg::OccupancyGrid &costmap,
    const nav_msgs::msg::OccupancyGrid &thermal,
    double rx, double ry) const {
    for (auto &f : candidates)
        f.corridor_gain = estimateGain(costmap, thermal, rx, ry, f.world_x, f.world_y);

    return *std::max_element(
        candidates.begin(), candidates.end(),
        [](const Frontier &a, const Frontier &b) {
            return a.corridor_gain < b.corridor_gain;
        });
}

double Explorer::computeCoverageRatio(
    const nav_msgs::msg::OccupancyGrid &spatial,
    const nav_msgs::msg::OccupancyGrid &thermal) const {
    const auto &si = spatial.info;
    const auto &ti = thermal.info;
    int total = 0, covered = 0;

    for (uint32_t row = 0; row < si.height; ++row) {
        for (uint32_t col = 0; col < si.width; ++col) {
            if (spatial.data[static_cast<size_t>(row) * si.width + col] != 0)
                continue;
            ++total;

            const double wx = si.origin.position.x + (col + 0.5) * si.resolution;
            const double wy = si.origin.position.y + (row + 0.5) * si.resolution;
            const int tc = static_cast<int>((wx - ti.origin.position.x) / ti.resolution);
            const int tr = static_cast<int>((wy - ti.origin.position.y) / ti.resolution);
            if (tc < 0 || tc >= (int)ti.width || tr < 0 || tr >= (int)ti.height)
                continue;
            if (thermal.data[static_cast<size_t>(tr) * ti.width + tc] >= 0)
                ++covered;
        }
    }
    return total == 0 ? 0.0 : static_cast<double>(covered) / total;
}

double Explorer::estimateGain(
    const nav_msgs::msg::OccupancyGrid &costmap,
    const nav_msgs::msg::OccupancyGrid &thermal,
    double rx, double ry, double gx, double gy) const {
    const auto &ci = costmap.info;
    const auto &ti = thermal.info;
    const double dx = gx - rx, dy = gy - ry;
    const double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 1e-6)
        return 0.0;

    const double res = ci.resolution;
    const int steps = std::max(1, static_cast<int>(dist / res));
    const int r_cells = static_cast<int>(
                            default_params_.sensor_coverage_radius / res) +
                        1;

    std::vector<bool> counted(static_cast<size_t>(ci.width) * ci.height, false);
    double gain = 0.0;

    for (int step = 0; step <= steps; ++step) {
        const double t = static_cast<double>(step) / steps;
        const double wx = rx + t * dx;
        const double wy = ry + t * dy;
        const int cx = static_cast<int>((wx - ci.origin.position.x) / res);
        const int cy = static_cast<int>((wy - ci.origin.position.y) / res);

        for (int dr = -r_cells; dr <= r_cells; ++dr) {
            for (int dc = -r_cells; dc <= r_cells; ++dc) {
                if (dr * dr + dc * dc > r_cells * r_cells)
                    continue;
                const int nr = cy + dr, nc = cx + dc;
                if (nr < 0 || nr >= (int)ci.height ||
                    nc < 0 || nc >= (int)ci.width)
                    continue;
                const size_t cidx = static_cast<size_t>(nr) * ci.width + nc;
                if (counted[cidx])
                    continue;
                if (costmap.data[cidx] < 0 || costmap.data[cidx] >= 99)
                    continue;

                const double cwx = ci.origin.position.x + (nc + 0.5) * res;
                const double cwy = ci.origin.position.y + (nr + 0.5) * res;
                const int tc = static_cast<int>((cwx - ti.origin.position.x) / ti.resolution);
                const int tr = static_cast<int>((cwy - ti.origin.position.y) / ti.resolution);
                if (tc < 0 || tc >= (int)ti.width || tr < 0 || tr >= (int)ti.height) {
                    counted[cidx] = true;
                    gain += 1.0;
                    continue;
                }
                if (thermal.data[static_cast<size_t>(tr) * ti.width + tc] == -1) {
                    counted[cidx] = true;
                    gain += 1.0;
                }
            }
        }
    }
    return gain;
}

void Explorer::sendGoal(double x, double y) {
    _current_goal_x = x;
    _current_goal_y = y;
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.stamp = ctx_.clock->now();
    goal.pose.header.frame_id = ctx_.map_frame;
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;
    ctx_.nav_client->async_send_goal(goal, ctx_.send_goal_options);
    ctx_.goal_active = true;
    ctx_.goal_sent_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(ctx_.logger, "[Explorer] Goal -> (%.2f,%.2f)", x, y);
}

void Explorer::checkTimeout() {
    if (!ctx_.goal_active)
        return;
    const double e = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - ctx_.goal_sent_time)
                         .count();
    if (e > default_params_.goal_timeout_seconds) {
        RCLCPP_WARN(ctx_.logger, "[Explorer] Timeout %.1fs", e);
        ctx_.nav_client->async_cancel_all_goals();
        ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;
        state_ = State::SCANNING;
    }
}

void Explorer::publishGoalMarker() {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker m;
    m.header.stamp = ctx_.clock->now();
    m.header.frame_id = ctx_.map_frame;
    m.ns = "visited_frontiers";
    m.id = ctx_.marker_id++;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = _current_goal_x;
    m.pose.position.y = _current_goal_y;
    m.pose.position.z = 0.05;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.12;
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.0f;
    m.color.a = 0.8f;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(m);
    ctx_.goal_marker_pub->publish(ma);
}

std::optional<std::pair<double, double>> Explorer::getRobotPose() const {
    try {
        const auto t = ctx_.tf_buffer->lookupTransform(
            ctx_.map_frame, ctx_.robot_frame,
            rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        return std::make_pair(
            t.transform.translation.x, t.transform.translation.y);
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(ctx_.logger, *ctx_.clock, 2000,
                             "[Explorer] TF: %s", e.what());
        return std::nullopt;
    }
}
Actor::Actor(NodeContext &ctx, const Params &p)
    : ctx_(ctx), params_(p) {}

bool Actor::update() {
    if (complete_)
        return true;
    switch (state_) {
    case State::PLANNING:
        handlePlanning();
        break;
    case State::NAVIGATING:
        handleNavigating();
        break;
    case State::ACTIONING:
        handleActioning();
        break;
    }
    return complete_;
}

void Actor::handlePlanning() {
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_copy;
    {
        std::lock_guard<std::mutex> lk(*ctx_.map_mutex);
        thermal_copy = ctx_.thermal_map;
    }
    if (!thermal_copy)
        return;

    zones_ = clusterHotSpots(*thermal_copy);
    if (zones_.empty()) {
        RCLCPP_WARN(ctx_.logger, "[Actor] No hot spots -- done");
        complete_ = true;
        return;
    }

    action_grid_.Initialize(*thermal_copy);

    const auto pose = getRobotPose();
    route_ = planRoute(zones_,
                       pose.has_value() ? pose->first : 0.0,
                       pose.has_value() ? pose->second : 0.0);
    current_idx_ = 0;

    RCLCPP_INFO(ctx_.logger, "[Actor] %zu zones", zones_.size());
    ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;

    state_ = State::NAVIGATING;
}

void Actor::handleNavigating() {
    if (current_idx_ >= route_.size()) {
        complete_ = true;
        return;
    }

    if (ctx_.goal_failed) {
        ctx_.goal_failed = false;
        RCLCPP_WARN(ctx_.logger, "[Actor] Zone %zu failed -- skipping",
                    current_idx_ + 1);
        ++current_idx_;
        ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;
        if (current_idx_ < route_.size()) {
            const auto &z = zones_[route_[current_idx_]];
            sendGoal(z.world_x, z.world_y);
        }
        return;
    }

    if (ctx_.goal_succeeded) {
        ctx_.goal_succeeded = false;
        action_start_ = std::chrono::steady_clock::now();
        state_ = State::ACTIONING;
        return;
    }

    if (!ctx_.goal_active) {
        const auto &z = zones_[route_[current_idx_]];
        publishZoneMarker(z, static_cast<int>(current_idx_));
        sendGoal(z.world_x, z.world_y);
    }
}

void Actor::handleActioning() {
    const double e = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - action_start_)
                         .count();
    if (e < params_.action_delay)
        return;

    const auto &zone = zones_[route_[current_idx_]];
    publishActionMap(zone);

    RCLCPP_INFO(ctx_.logger, "[Actor] Zone %zu actioned", current_idx_ + 1);
    ++current_idx_;
    ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;
    state_ = State::NAVIGATING;
}

std::vector<Actor::ActionZone> Actor::clusterHotSpots(
    const nav_msgs::msg::OccupancyGrid &thermal) const {
    const auto &ti = thermal.info;
    const auto thresh = static_cast<int8_t>(params_.heat_threshold);
    const int cr = static_cast<int>(
        params_.cluster_radius / ti.resolution);

    struct HotCell {
        int row, col;
        int8_t value;
    };
    std::vector<HotCell> hot;
    hot.reserve(1024);

    for (uint32_t row = 0; row < ti.height; ++row)
        for (uint32_t col = 0; col < ti.width; ++col) {
            const auto v =
                thermal.data[static_cast<size_t>(row) * ti.width + col];
            if (v >= thresh)
                hot.push_back({(int)row, (int)col, v});
        }

    if (hot.empty())
        return {};

    std::sort(hot.begin(), hot.end(),
              [](const HotCell &a, const HotCell &b) { return a.value > b.value; });

    std::vector<bool> assigned(hot.size(), false);
    std::vector<ActionZone> zones;

    for (size_t i = 0; i < hot.size(); ++i) {
        if (assigned[i])
            continue;
        double sx = 0, sy = 0, sh = 0;
        int cnt = 0;
        for (size_t j = i; j < hot.size(); ++j) {
            if (assigned[j])
                continue;
            const int dr = hot[j].row - hot[i].row;
            const int dc = hot[j].col - hot[i].col;
            if (std::abs(dr) > cr || std::abs(dc) > cr)
                continue;
            if (std::sqrt((double)(dr * dr + dc * dc)) * ti.resolution >
                params_.cluster_radius)
                continue;
            assigned[j] = true;
            sx += ti.origin.position.x + (hot[j].col + 0.5) * ti.resolution;
            sy += ti.origin.position.y + (hot[j].row + 0.5) * ti.resolution;
            sh += hot[j].value;
            ++cnt;
        }
        zones.push_back({sx / cnt, sy / cnt, sh / cnt});
    }
    return zones;
}

std::vector<size_t> Actor::planRoute(
    const std::vector<ActionZone> &zones,
    double rx, double ry) const {
    std::vector<size_t> route;
    std::vector<bool> visited(zones.size(), false);
    double cx = rx, cy = ry;
    for (size_t step = 0; step < zones.size(); ++step) {
        double best = std::numeric_limits<double>::max();
        size_t bidx = 0;
        for (size_t i = 0; i < zones.size(); ++i) {
            if (visited[i])
                continue;
            const double dx = zones[i].world_x - cx;
            const double dy = zones[i].world_y - cy;
            const double d = dx * dx + dy * dy;
            if (d < best) {
                best = d;
                bidx = i;
            }
        }
        visited[bidx] = true;
        route.push_back(bidx);
        cx = zones[bidx].world_x;
        cy = zones[bidx].world_y;
    }
    return route;
}

void Actor::publishZoneMarker(const ActionZone &zone, int id) {
    visualization_msgs::msg::MarkerArray ma;
    visualization_msgs::msg::Marker m;
    m.header.stamp = ctx_.clock->now();
    m.header.frame_id = ctx_.map_frame;
    m.ns = "action_zones";
    m.id = id;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = zone.world_x;
    m.pose.position.y = zone.world_y;
    m.pose.position.z = 0.15;
    m.pose.orientation.w = 1.0;
    const double sz = 0.15 + (zone.strength / 100.0) * 0.25;
    m.scale.x = m.scale.y = m.scale.z = sz;
    m.color.r = 0.0f;
    m.color.g = 1.0f;
    m.color.b = 0.3f;
    m.color.a = 0.9f;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(m);
    ctx_.zone_marker_pub->publish(ma);
}

void Actor::publishActionMap(const ActionZone &zone) {
    action_grid_.WriteZone(
        zone.world_x, zone.world_y,
        zone.strength, params_.base_sigma);
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = ctx_.clock->now();
    msg.header.frame_id = ctx_.map_frame;
    msg.info = action_grid_.getInfo();
    msg.data = action_grid_.ToOccupancyData();
    ctx_.action_map_pub->publish(msg);
}

void Actor::sendGoal(double x, double y) {
    const auto pose = getRobotPose();
    if (pose.has_value()) {
        const double dx = pose->first - x;
        const double dy = pose->second - y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 0.35) {
            x += (dx / dist) * 0.35;
            y += (dy / dist) * 0.35;
        }
    }

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.stamp = ctx_.clock->now();
    goal.pose.header.frame_id = ctx_.map_frame;
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;

    ctx_.nav_client->async_send_goal(goal, ctx_.send_goal_options);
    ctx_.goal_active = true;
    ctx_.goal_sent_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(ctx_.logger, "[Actor] Goal -> (%.2f,%.2f)", x, y);
}

std::optional<std::pair<double, double>> Actor::getRobotPose() const {
    try {
        const auto t = ctx_.tf_buffer->lookupTransform(
            ctx_.map_frame, ctx_.robot_frame,
            rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        return std::make_pair(
            t.transform.translation.x, t.transform.translation.y);
    } catch (const tf2::TransformException &e) {
        RCLCPP_WARN_THROTTLE(ctx_.logger, *ctx_.clock, 2000,
                             "[Actor] TF: %s", e.what());
        return std::nullopt;
    }
}

DecisionNode::DecisionNode() : Node("decision_node") {
    declare_parameter("map_frame", std::string("map"));
    declare_parameter("robot_frame", std::string("base_footprint"));
    declare_parameter("coverage_threshold", 0.95);
    declare_parameter("sensor_coverage_radius", 0.3);
    declare_parameter("goal_min_distance", 0.5);
    declare_parameter("goal_timeout_seconds", 30.0);
    declare_parameter("rescan_interval_seconds", 8.0);
    declare_parameter("radius_initial", 1.5);
    declare_parameter("radius_step", 0.5);
    declare_parameter("radius_max", 8.0);
    declare_parameter("samples_per_cycle", 40);
    declare_parameter("corridor_bonus", 0.3);

    declare_parameter("min_frontier_size", 3);
    declare_parameter("action_zone_heat_threshold", 60.0);
    declare_parameter("action_zone_cluster_radius", 1.5);
    declare_parameter("action_zone_base_sigma", 0.4);
    declare_parameter("action_delay_seconds", 1.0);
    declare_parameter("control_rate", 1.0);

    const std::string map_frame = get_parameter("map_frame").as_string();
    const std::string robot_frame = get_parameter("robot_frame").as_string();
    const double control_rate = get_parameter("control_rate").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    nav_client_ = rclcpp_action::create_client<
        nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    rclcpp::QoS latch(1);
    latch.transient_local().reliable();

    map_mutex_ = std::make_shared<std::mutex>();

    ctx_.logger = get_logger();
    ctx_.clock = get_clock();
    ctx_.tf_buffer = tf_buffer_;
    ctx_.nav_client = nav_client_;
    ctx_.map_mutex = map_mutex_;
    ctx_.map_frame = map_frame;
    ctx_.robot_frame = robot_frame;
    ctx_.action_map_pub =
        create_publisher<nav_msgs::msg::OccupancyGrid>("/action_map", latch);
    ctx_.goal_marker_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/thermocator/goal_markers",
            rclcpp::QoS(1).transient_local().reliable());
    ctx_.zone_marker_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/thermocator/action_zones",
            rclcpp::QoS(1).transient_local().reliable());

    ctx_.send_goal_options.goal_response_callback =
        std::bind(&DecisionNode::goalResponseCallback, this,
                  std::placeholders::_1);
    ctx_.send_goal_options.feedback_callback =
        std::bind(&DecisionNode::feedbackCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    ctx_.send_goal_options.result_callback =
        std::bind(&DecisionNode::resultCallback, this,
                  std::placeholders::_1);

    thermal_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/thermal_map", latch,
        std::bind(&DecisionNode::thermalMapCallback, this,
                  std::placeholders::_1));
    spatial_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", latch,
        std::bind(&DecisionNode::spatialMapCallback, this,
                  std::placeholders::_1));
    costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", latch,
        std::bind(&DecisionNode::costmapCallback, this,
                  std::placeholders::_1));

    Explorer::Params ep;
    ep.coverage_threshold = get_parameter("coverage_threshold").as_double();
    ep.sensor_coverage_radius = get_parameter("sensor_coverage_radius").as_double();
    ep.goal_min_distance = get_parameter("goal_min_distance").as_double();
    ep.goal_timeout_seconds = get_parameter("goal_timeout_seconds").as_double();
    ep.rescan_interval_seconds = get_parameter("rescan_interval_seconds").as_double();
    ep.radius_initial = get_parameter("radius_initial").as_double();
    ep.radius_step = get_parameter("radius_step").as_double();
    ep.radius_max = get_parameter("radius_max").as_double();
    ep.samples_per_cycle = get_parameter("samples_per_cycle").as_int();
    ep.corridor_bonus = get_parameter("corridor_bonus").as_double();
    explorer_ = std::make_unique<Explorer>(ctx_, ep);

    Actor::Params ap;
    ap.heat_threshold = get_parameter("action_zone_heat_threshold").as_double();
    ap.cluster_radius = get_parameter("action_zone_cluster_radius").as_double();
    ap.base_sigma = get_parameter("action_zone_base_sigma").as_double();
    ap.action_delay = get_parameter("action_delay_seconds").as_double();
    actor_ = std::make_unique<Actor>(ctx_, ap);

    const auto period = std::chrono::duration<double>(1.0 / control_rate);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DecisionNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "DecisionNode ready");
}

void DecisionNode::thermalMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(*map_mutex_);
    ctx_.thermal_map = msg;
}

void DecisionNode::spatialMapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(*map_mutex_);
    ctx_.spatial_map = msg;
}

void DecisionNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(*map_mutex_);
    ctx_.costmap = msg;
}

void DecisionNode::controlLoop() {
    if (phase_ == Phase::WAITING) {
        bool ready;
        {
            std::lock_guard<std::mutex> lk(*map_mutex_);
            ready = ctx_.thermal_map && ctx_.spatial_map && ctx_.costmap;
        }
        if (!ready) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for maps and costmap ...");
            return;
        }
        if (!nav_client_->wait_for_action_server(
                std::chrono::milliseconds(100))) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for Nav2 ...");
            return;
        }
        RCLCPP_INFO(get_logger(), "Ready -- Phase 1");
        phase_ = Phase::PHASE1;
    }

    if (phase_ == Phase::PHASE1) {
        if (explorer_->update()) {
            RCLCPP_INFO(get_logger(), "Phase 1 done -- Phase 2");
            phase_ = Phase::PHASE2;
        }
        return;
    }

    if (phase_ == Phase::PHASE2) {
        if (actor_->update()) {
            RCLCPP_INFO(get_logger(), "Phase 2 done -- mission complete");
            phase_ = Phase::DONE;
        }
        return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                         "Mission complete");
}

void DecisionNode::goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr &handle) {
    if (!handle) {
        ctx_.goal_active = false;
        ctx_.goal_failed = true;
        RCLCPP_WARN(get_logger(), "Goal rejected");
        return;
    }
    current_goal_handle_ = handle;
}

void DecisionNode::feedbackCallback(
    rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<
        const nav2_msgs::action::NavigateToPose::Feedback>
        fb) {
    RCLCPP_DEBUG(get_logger(),
                 "Remaining: %.2fm", fb->distance_remaining);
}

void DecisionNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::WrappedResult &result) {
    ctx_.goal_active = false;
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        ctx_.goal_succeeded = true;
    else
        ctx_.goal_failed = true;
}

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
