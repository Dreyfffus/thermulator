#include "thermocator/decision_node.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <queue>
#include <rclcpp/logging.hpp>
#include <string>

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
        setGoal(best.world_x, best.world_y, best.corridor_gain);
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
    if (ctx_.goal_failed) {
        ctx_.goal_failed = false;
        RCLCPP_WARN(ctx_.logger, "[Explorer] Goal failed/timed out (%.2f,%.2f)",
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

void Explorer::setGoal(double x, double y, double score) {
    _current_goal_x = x;
    _current_goal_y = y;
    ctx_.current_goal_x = x;
    ctx_.current_goal_y = y;
    ctx_.current_score = score;
    ctx_.goal_active = true;
    ctx_.goal_sent_time = std::chrono::steady_clock::now();
    RCLCPP_INFO(ctx_.logger, "[Explorer] Candidate -> (%.2f,%.2f) score=%.1f src=%s",
                x, y, score, ctx_.goal_source.c_str());
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
    // Wait until the decision node has installed the agreed (finalized) plan.
    if (!has_plan_)
        return;

    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_copy;
    {
        std::lock_guard<std::mutex> lk(*ctx_.map_mutex);
        thermal_copy = ctx_.thermal_map;
    }
    if (!thermal_copy)
        return;

    // The plan is already merged, nudged and ordered by the coordinator. Both
    // sides execute it VERBATIM (same points, same order) -- no re-nudging and
    // no re-routing -- so the robot and twin run the identical plan.
    zones_ = pending_zones_;
    if (zones_.empty()) {
        RCLCPP_WARN(ctx_.logger, "[Actor] Agreed plan is empty -- done");
        complete_ = true;
        return;
    }

    action_grid_.Initialize(*thermal_copy);

    route_.resize(zones_.size());
    std::iota(route_.begin(), route_.end(), 0); // execute in the agreed order
    current_idx_ = 0;

    RCLCPP_INFO(ctx_.logger, "[Actor] actioning %zu agreed zones (shared plan)",
                zones_.size());
    ctx_.goal_active = ctx_.goal_succeeded = ctx_.goal_failed = false;

    state_ = State::NAVIGATING;
}

std::vector<ActionZone> Actor::computeZones(
    const nav_msgs::msg::OccupancyGrid &costmap,
    const nav_msgs::msg::OccupancyGrid &thermal) const {
    return clusterHotSpots(costmap, thermal);
}

std::vector<ActionZone> Actor::finalizePlan(
    const std::vector<ActionZone> &merged,
    const nav_msgs::msg::OccupancyGrid &costmap,
    double rx, double ry) const {
    // Nudge every merged zone to a navigable cell, then order into a route.
    std::vector<ActionZone> nudged;
    nudged.reserve(merged.size());
    for (const auto &z : merged) {
        const auto [nx, ny] = nudgeToFreeCell(z.world_x, z.world_y, costmap);
        nudged.push_back({nx, ny, z.strength});
    }

    const auto order = planRoute(nudged, rx, ry);
    std::vector<ActionZone> ordered;
    ordered.reserve(nudged.size());
    for (size_t idx : order)
        ordered.push_back(nudged[idx]);
    return ordered;
}

void Actor::setPlan(const std::vector<ActionZone> &zones) {
    pending_zones_ = zones; // already finalized + ordered by the coordinator
    has_plan_ = true;
    complete_ = false;
    current_idx_ = 0;
    state_ = State::PLANNING;
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
            setGoal(z.world_x, z.world_y, z.strength);
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
        setGoal(z.world_x, z.world_y, z.strength);
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

std::vector<ActionZone> Actor::clusterHotSpots(
    const nav_msgs::msg::OccupancyGrid &costmap,
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

        const double raw_x = sx / cnt;
        const double raw_y = sy / cnt;

        const auto [nav_x, nav_y] = nudgeToFreeCell(raw_x, raw_y, costmap);

        zones.push_back({nav_x, nav_y, sh / cnt});
    }

    return zones;
}

std::pair<double, double> Actor::nudgeToFreeCell(
    double wx, double wy,
    const nav_msgs::msg::OccupancyGrid &costmap) const {
    const auto &ci = costmap.info;

    const int col0 = static_cast<int>((wx - ci.origin.position.x) / ci.resolution);
    const int row0 = static_cast<int>((wy - ci.origin.position.y) / ci.resolution);

    if (col0 >= 0 && col0 < (int)ci.width &&
        row0 >= 0 && row0 < (int)ci.height) {
        const auto cv = costmap.data[static_cast<size_t>(row0) * ci.width + col0];
        if (cv >= 0 && cv < 99)
            return {wx, wy};
    }

    const int start_col = std::clamp(col0, 0, (int)ci.width - 1);
    const int start_row = std::clamp(row0, 0, (int)ci.height - 1);

    const size_t map_sz = static_cast<size_t>(ci.width) * ci.height;
    std::vector<bool> visited(map_sz, false);
    std::queue<std::pair<int, int>> q;

    const size_t start_idx = static_cast<size_t>(start_row) * ci.width + start_col;
    visited[start_idx] = true;
    q.push({start_row, start_col});

    const int nb8[8][2] = {
        {-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    const int max_search_cells = params_.max_search_cells;
    int searched = 0;

    while (!q.empty() && searched < max_search_cells) {
        const auto [r, c] = q.front();
        q.pop();
        ++searched;

        const auto cv = costmap.data[static_cast<size_t>(r) * ci.width + c];

        if (cv >= 0 && cv < 99) {
            return {
                ci.origin.position.x + (c + 0.5) * ci.resolution,
                ci.origin.position.y + (r + 0.5) * ci.resolution};
        }

        for (const auto &n : nb8) {
            const int nr = r + n[0], nc = c + n[1];
            if (nr < 0 || nr >= (int)ci.height ||
                nc < 0 || nc >= (int)ci.width)
                continue;
            const size_t nidx = static_cast<size_t>(nr) * ci.width + nc;
            if (!visited[nidx]) {
                visited[nidx] = true;
                q.push({nr, nc});
            }
        }
    }

    RCLCPP_WARN_THROTTLE(ctx_.logger, *ctx_.clock, 3000,
                         "[Actor] no free cell near (%.2f, %.2f) within %d steps -- using original",
                         wx, wy, max_search_cells);

    return {wx, wy};
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

void Actor::setGoal(double x, double y, double score) {
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

    ctx_.current_goal_x = x;
    ctx_.current_goal_y = y;
    ctx_.current_score = score;
    ctx_.goal_active = true;
    ctx_.goal_sent_time = std::chrono::steady_clock::now();

    RCLCPP_INFO(ctx_.logger, "[Actor] Candidate -> (%.2f,%.2f) score=%.1f src=%s",
                x, y, score, ctx_.goal_source.c_str());
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
    declare_parameter("goal_source", std::string("LOCAL"));
    declare_parameter("arrival_radius", 0.4);
    declare_parameter("merge_dedup_radius", 1.5);
    declare_parameter("coverage_threshold", 0.95);
    declare_parameter("sensor_coverage_radius", 0.3);
    declare_parameter("goal_min_distance", 0.5);
    declare_parameter("goal_timeout_seconds", 70.0);
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
    declare_parameter("max_search_cells", 200);

    const std::string map_frame = get_parameter("map_frame").as_string();
    const std::string robot_frame = get_parameter("robot_frame").as_string();
    const double control_rate = get_parameter("control_rate").as_double();
    goal_timeout_seconds_ = get_parameter("goal_timeout_seconds").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS latch(1);
    latch.transient_local().reliable();

    map_mutex_ = std::make_shared<std::mutex>();

    ctx_.logger = get_logger();
    ctx_.clock = get_clock();
    ctx_.tf_buffer = tf_buffer_;
    ctx_.map_mutex = map_mutex_;
    ctx_.map_frame = map_frame;
    ctx_.robot_frame = robot_frame;
    ctx_.goal_source = get_parameter("goal_source").as_string();
    ctx_.arrival_radius = get_parameter("arrival_radius").as_double();
    merge_dedup_radius_ = get_parameter("merge_dedup_radius").as_double();

    ctx_.goal_pub = create_publisher<thermocator_msgs::msg::GoalCandidate>(
        "/thermocator/goals", rclcpp::QoS(10).reliable());
    ctx_.action_map_pub =
        create_publisher<nav_msgs::msg::OccupancyGrid>("/action_map", latch);
    ctx_.zone_marker_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>(
            "/thermocator/action_zones",
            rclcpp::QoS(1).transient_local().reliable());

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

    // Mission-state coordination. Each side publishes on its own source topic and
    // listens to the peer's, so the (bidirectional) domain bridge never loops.
    const bool is_local = ctx_.goal_source != "TWINNED";
    const std::string own_topic =
        is_local ? "/thermocator/state/local" : "/thermocator/state/twinned";
    const std::string peer_topic =
        is_local ? "/thermocator/state/twinned" : "/thermocator/state/local";

    state_pub_ = create_publisher<thermocator_msgs::msg::MissionState>(
        own_topic, rclcpp::QoS(10).reliable());
    peer_state_sub_ = create_subscription<thermocator_msgs::msg::MissionState>(
        peer_topic, rclcpp::QoS(10).reliable(),
        [this](thermocator_msgs::msg::MissionState::SharedPtr msg) {
            peer_state_ = msg;
        });

    Explorer::Params ep;
    ep.coverage_threshold = get_parameter("coverage_threshold").as_double();
    ep.sensor_coverage_radius = get_parameter("sensor_coverage_radius").as_double();
    ep.goal_min_distance = get_parameter("goal_min_distance").as_double();
    ep.goal_timeout_seconds = goal_timeout_seconds_;
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
    ap.max_search_cells = get_parameter("max_search_cells").as_int();
    actor_ = std::make_unique<Actor>(ctx_, ap);

    const auto period = std::chrono::duration<double>(1.0 / control_rate);
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&DecisionNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "DecisionNode ready -- source=%s",
                ctx_.goal_source.c_str());
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

void DecisionNode::updateGoalStatus() {
    if (!ctx_.goal_active)
        return;

    std::optional<std::pair<double, double>> pose;
    try {
        const auto t = tf_buffer_->lookupTransform(
            ctx_.map_frame, ctx_.robot_frame,
            rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        pose = std::make_pair(t.transform.translation.x, t.transform.translation.y);
    } catch (const tf2::TransformException &) {
        pose = std::nullopt;
    }

    if (pose.has_value()) {
        const double dx = pose->first - ctx_.current_goal_x;
        const double dy = pose->second - ctx_.current_goal_y;
        if (std::sqrt(dx * dx + dy * dy) <= ctx_.arrival_radius) {
            ctx_.goal_succeeded = true;
            ctx_.goal_active = false;
            return;
        }
    }

    const double elapsed = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - ctx_.goal_sent_time)
                               .count();
    if (elapsed > goal_timeout_seconds_) {
        RCLCPP_WARN(get_logger(), "[Decision] goal timeout %.1fs", elapsed);
        ctx_.goal_failed = true;
        ctx_.goal_active = false;
    }
}

void DecisionNode::publishCurrentCandidate() {
    thermocator_msgs::msg::GoalCandidate cand;
    cand.pose.header.stamp = now();
    cand.pose.header.frame_id = ctx_.map_frame;
    cand.pose.pose.position.x = ctx_.current_goal_x;
    cand.pose.pose.position.y = ctx_.current_goal_y;
    cand.pose.pose.orientation.w = 1.0;
    cand.source = ctx_.goal_source;
    cand.score = ctx_.current_score;
    ctx_.goal_pub->publish(cand);
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
        RCLCPP_INFO(get_logger(), "Ready -- Phase 1 (Explore)");
        phase_ = Phase::PHASE1;
    }

    // React to the peer first: it may pull us into Act or Done.
    handlePeerState();

    updateGoalStatus();

    if (phase_ == Phase::PHASE1) {
        const bool explore_done = explorer_->update();
        if (explore_done && !plan_adopted_) {
            // We finished exploring first -> become the Act coordinator.
            triggerActTransition();
        }
    } else if (phase_ == Phase::PHASE2) {
        if (actor_->update()) {
            RCLCPP_INFO(get_logger(), "Phase 2 done -- broadcasting DONE");
            enterDone();
        }
    } else {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 30000,
                             "Mission complete");
    }

    broadcastState();

    // Heartbeat: keep the arbiter's view of this proposer's goal fresh.
    if (phase_ != Phase::DONE && ctx_.goal_active)
        publishCurrentCandidate();
}

void DecisionNode::handlePeerState() {
    if (!peer_state_ || phase_ == Phase::DONE)
        return;

    const auto &p = *peer_state_;

    if (p.phase == thermocator_msgs::msg::MissionState::PHASE_DONE) {
        RCLCPP_INFO(get_logger(), "[Sync] peer %s finished -- finishing too",
                    p.source.c_str());
        enterDone();
        return;
    }

    if (p.phase == thermocator_msgs::msg::MissionState::PHASE_ACT && p.plan) {
        if (!plan_adopted_) {
            adoptPlan(p.source, peerZones());
        } else if (plan_author_ != "LOCAL" && p.source == "LOCAL") {
            // Simultaneous trigger: LOCAL's plan wins the tie-break.
            RCLCPP_INFO(get_logger(),
                        "[Sync] adopting LOCAL plan over our own (tie-break)");
            adoptPlan(p.source, peerZones());
        }
    }
}

void DecisionNode::triggerActTransition() {
    const auto own = currentZones();
    const auto merged = mergeZones(own, peerZones());

    // Finalize the canonical plan ONCE (nudge + order) so both sides execute the
    // exact same ordered points.
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_copy;
    {
        std::lock_guard<std::mutex> lk(*map_mutex_);
        costmap_copy = ctx_.costmap;
    }
    const auto pose = robotPose();
    const double rx = pose.has_value() ? pose->first : 0.0;
    const double ry = pose.has_value() ? pose->second : 0.0;

    std::vector<ActionZone> agreed =
        costmap_copy ? actor_->finalizePlan(merged, *costmap_copy, rx, ry) : merged;

    plan_adopted_ = true;
    plan_author_ = ctx_.goal_source;
    agreed_zones_ = agreed;
    actor_->setPlan(agreed);
    phase_ = Phase::PHASE2;

    RCLCPP_INFO(get_logger(),
                "[Sync] Explore done -- I am Act coordinator: %zu own + peer "
                "-> %zu agreed zones (finalized, shared verbatim)",
                own.size(), agreed.size());
}

void DecisionNode::adoptPlan(const std::string &author,
                             const std::vector<ActionZone> &zones) {
    plan_adopted_ = true;
    plan_author_ = author;
    agreed_zones_ = zones;
    actor_->setPlan(zones);
    phase_ = Phase::PHASE2;
    RCLCPP_INFO(get_logger(), "[Sync] adopted %s plan: %zu zones -- Phase 2",
                author.c_str(), zones.size());
}

void DecisionNode::enterDone() {
    phase_ = Phase::DONE;
    ctx_.goal_active = false;
}

std::vector<ActionZone> DecisionNode::currentZones() {
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_copy;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_copy;
    {
        std::lock_guard<std::mutex> lk(*map_mutex_);
        thermal_copy = ctx_.thermal_map;
        costmap_copy = ctx_.costmap;
    }
    if (!thermal_copy || !costmap_copy)
        return cached_zones_;

    // Throttle the (whole-grid) clustering to ~3 s during exploration.
    const auto now_t = std::chrono::steady_clock::now();
    const double age =
        std::chrono::duration<double>(now_t - last_zone_calc_).count();
    if (!zones_ever_calced_ || age > 3.0) {
        cached_zones_ = actor_->computeZones(*costmap_copy, *thermal_copy);
        last_zone_calc_ = now_t;
        zones_ever_calced_ = true;
    }
    return cached_zones_;
}

std::vector<ActionZone> DecisionNode::peerZones() const {
    std::vector<ActionZone> out;
    if (!peer_state_)
        return out;
    const auto &p = *peer_state_;
    const size_t n = p.zones.size();
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const double s = (i < p.strengths.size()) ? p.strengths[i] : 0.0;
        out.push_back({p.zones[i].x, p.zones[i].y, s});
    }
    return out;
}

std::vector<ActionZone> DecisionNode::mergeZones(
    const std::vector<ActionZone> &own,
    const std::vector<ActionZone> &peer) const {
    std::vector<ActionZone> merged = own;
    const double r2 = merge_dedup_radius_ * merge_dedup_radius_;
    for (const auto &pz : peer) {
        bool dup = false;
        for (const auto &mz : merged) {
            const double dx = pz.world_x - mz.world_x;
            const double dy = pz.world_y - mz.world_y;
            if (dx * dx + dy * dy <= r2) {
                dup = true;
                break;
            }
        }
        if (!dup)
            merged.push_back(pz);
    }
    return merged;
}

std::optional<std::pair<double, double>> DecisionNode::robotPose() const {
    try {
        const auto t = tf_buffer_->lookupTransform(
            ctx_.map_frame, ctx_.robot_frame,
            rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
        return std::make_pair(t.transform.translation.x,
                              t.transform.translation.y);
    } catch (const tf2::TransformException &) {
        return std::nullopt;
    }
}

void DecisionNode::broadcastState() {
    thermocator_msgs::msg::MissionState m;
    m.header.stamp = now();
    m.header.frame_id = ctx_.map_frame;
    m.source = ctx_.goal_source;

    if (phase_ == Phase::DONE) {
        m.phase = thermocator_msgs::msg::MissionState::PHASE_DONE;
        m.plan = false;
    } else if (phase_ == Phase::PHASE2) {
        m.phase = thermocator_msgs::msg::MissionState::PHASE_ACT;
        // The plan author keeps re-advertising the agreed zones so a peer that
        // started late (or missed the first message) still converges on them.
        if (plan_author_ == ctx_.goal_source) {
            m.plan = true;
            for (const auto &z : agreed_zones_) {
                geometry_msgs::msg::Point pt;
                pt.x = z.world_x;
                pt.y = z.world_y;
                m.zones.push_back(pt);
                m.strengths.push_back(z.strength);
            }
        } else {
            m.plan = false;
        }
    } else {
        // Explore: advertise our current detections so the eventual coordinator
        // can merge them into the agreed plan.
        m.phase = thermocator_msgs::msg::MissionState::PHASE_EXPLORE;
        m.plan = false;
        for (const auto &z : currentZones()) {
            geometry_msgs::msg::Point pt;
            pt.x = z.world_x;
            pt.y = z.world_y;
            m.zones.push_back(pt);
            m.strengths.push_back(z.strength);
        }
    }

    state_pub_->publish(m);
}

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
