#include "nav_msgs/msg/odometry.hpp"
#include <cmath>
#include <mutex>
#include <optional>
#include <random>
#include <rclcpp/subscription.hpp>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace thermocator {

class AdvisoryNode : public rclcpp::Node {
  public:
    AdvisoryNode() : Node("advisory_node") {
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("robot_frame", std::string("base_footprint"));
        declare_parameter("sensor_radius", 0.3);
        declare_parameter("goal_min_distance", 0.5);
        declare_parameter("coverage_threshold", 0.95);
        declare_parameter("radius_initial", 1.5);
        declare_parameter("radius_step", 0.5);
        declare_parameter("radius_max", 8.0);
        declare_parameter("samples_per_cycle", 40);
        declare_parameter("corridor_bonus", 0.3);
        declare_parameter("advisory_stale_secs", 2.0);
        map_frame_ = get_parameter("map_frame").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();
        sensor_radius_ = get_parameter("sensor_radius").as_double();
        goal_min_distance_ = get_parameter("goal_min_distance").as_double();
        coverage_threshold_ = get_parameter("coverage_threshold").as_double();
        radius_initial_ = get_parameter("radius_initial").as_double();
        radius_step_ = get_parameter("radius_step").as_double();
        radius_max_ = get_parameter("radius_max").as_double();
        samples_per_cycle_ = get_parameter("samples_per_cycle").as_int();
        corridor_bonus_ = get_parameter("corridor_bonus").as_double();
        advisory_stale_secs_ = get_parameter("advisory_stale_secs").as_double();

        sample_radius_ = radius_initial_;

        rclcpp::QoS latched(1);
        latched.transient_local().reliable();

        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/advisory/goal", rclcpp::QoS(10).reliable());

        marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/advisory/candidates", rclcpp::QoS(10).reliable());

        thermal_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/thermal_map", latched,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(map_mutex_);
                thermal_map_ = msg;
                map_dirty_ = true;
            });

        spatial_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", latched,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(map_mutex_);
                spatial_map_ = msg;
                map_dirty_ = true;
            });

        costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/global_costmap/costmap", latched,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(map_mutex_);
                costmap_ = msg;
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x_ = msg->pose.pose.position.x;
                robot_y_ = msg->pose.pose.position.y;
                robot_pose_valid_ = true;
            });

        eval_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&AdvisoryNode::evaluate, this));

        RCLCPP_INFO(get_logger(), "AdvisoryNode ready -- publishing on /advisory/goal");
    }

  private:
    struct Candidate {
        double world_x = 0.0;
        double world_y = 0.0;
        double corridor_gain = 0.0;
        double distance = 0.0;
    };

    void evaluate() {
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            if (!map_dirty_)
                return;
            map_dirty_ = false;
        }

        nav_msgs::msg::OccupancyGrid::SharedPtr thermal_copy;
        nav_msgs::msg::OccupancyGrid::SharedPtr spatial_copy;
        nav_msgs::msg::OccupancyGrid::SharedPtr costmap_copy;
        {
            std::lock_guard<std::mutex> lk(map_mutex_);
            thermal_copy = thermal_map_;
            spatial_copy = spatial_map_;
            costmap_copy = costmap_;
        }
        if (!thermal_copy || !spatial_copy || !costmap_copy)
            return;

        if (!robot_pose_valid_)
            return;
        const double rx = robot_x_;
        const double ry = robot_y_;

        const double cov = computeCoverageRatio(*spatial_copy, *thermal_copy);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                             "[Advisory] Coverage %.1f%%  radius=%.1fm",
                             cov * 100.0, sample_radius_);

        if (cov >= coverage_threshold_) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                                 "[Advisory] Coverage complete -- no advisory needed");
            return;
        }

        auto candidates = sampleCandidates(
            *thermal_copy, *costmap_copy, rx, ry);

        if (candidates.empty()) {
            sample_radius_ = std::min(sample_radius_ + radius_step_, radius_max_);
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "[Advisory] No candidates -- radius -> %.1fm",
                                 sample_radius_);
            return;
        }

        for (auto &c : candidates)
            c.corridor_gain = estimateCorridorGain(
                *costmap_copy, *thermal_copy,
                rx, ry,
                c.world_x, c.world_y);

        const auto &best = *std::max_element(
            candidates.begin(), candidates.end(),
            [this](const Candidate &a, const Candidate &b) {
                return (corridor_bonus_ * a.corridor_gain) <
                       (corridor_bonus_ * b.corridor_gain);
            });

        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = now();
        goal.header.frame_id = map_frame_;
        goal.pose.position.x = best.world_x;
        goal.pose.position.y = best.world_y;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.w = 1.0;

        goal_pub_->publish(goal);

        RCLCPP_INFO(get_logger(),
                    "[Advisory] Goal (%.2f,%.2f) cg=%.0f d=%.2f",
                    best.world_x, best.world_y,
                    best.corridor_gain, best.distance);

        publishCandidateMarkers(candidates, best);

        sample_radius_ = radius_initial_;
    }

    std::vector<Candidate> sampleCandidates(
        const nav_msgs::msg::OccupancyGrid &thermal,
        const nav_msgs::msg::OccupancyGrid &costmap,
        double rx, double ry) const {
        const auto &ci = costmap.info;
        const auto &ti = thermal.info;

        std::uniform_real_distribution<double> uniform(-sample_radius_, sample_radius_);

        std::vector<Candidate> valid;
        valid.reserve(samples_per_cycle_);

        const int max_attempts = samples_per_cycle_ * 4;

        for (int attempt = 0;
             attempt < max_attempts && (int)valid.size() < samples_per_cycle_;
             ++attempt) {

            const double ox = uniform(rng_);
            const double oy = uniform(rng_);
            const double d2 = ox * ox + oy * oy;
            if (d2 > sample_radius_ * sample_radius_)
                continue;

            const double dist = std::sqrt(d2);
            if (dist < goal_min_distance_)
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
                valid.push_back({wx, wy, 0.0, dist});
                continue;
            }
            if (thermal.data[static_cast<size_t>(tr) * ti.width + tc] == -1)
                valid.push_back({wx, wy, 0.0, dist});
        }

        return valid;
    }

    double estimateCorridorGain(
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
        const int r_cells = static_cast<int>(sensor_radius_ / res) + 1;

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

    double computeCoverageRatio(
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

    void publishCandidateMarkers(
        const std::vector<Candidate> &candidates,
        const Candidate &best) const {
        if (!marker_pub_)
            return;

        const double max_gain = best.corridor_gain + 1e-6;
        visualization_msgs::msg::MarkerArray ma;

        for (size_t i = 0; i < candidates.size(); ++i) {
            const auto &c = candidates[i];
            visualization_msgs::msg::Marker m;
            m.header.stamp = now();
            m.header.frame_id = map_frame_;
            m.ns = "advisory_candidates";
            m.id = static_cast<int>(i);
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = c.world_x;
            m.pose.position.y = c.world_y;
            m.pose.position.z = 0.05;
            m.pose.orientation.w = 1.0;
            m.scale.x = m.scale.y = m.scale.z = 0.08;
            const float norm = static_cast<float>(c.corridor_gain / max_gain);
            m.color.r = 1.0f - norm;
            m.color.g = norm;
            m.color.b = 0.0f;
            m.color.a = 0.7f;
            m.lifetime = rclcpp::Duration::from_seconds(2.0);
            ma.markers.push_back(m);
        }

        // Extra marker for the chosen best — blue sphere, larger
        visualization_msgs::msg::Marker bm;
        bm.header.stamp = now();
        bm.header.frame_id = map_frame_;
        bm.ns = "advisory_best";
        bm.id = 0;
        bm.type = visualization_msgs::msg::Marker::SPHERE;
        bm.action = visualization_msgs::msg::Marker::ADD;
        bm.pose.position.x = best.world_x;
        bm.pose.position.y = best.world_y;
        bm.pose.position.z = 0.15;
        bm.pose.orientation.w = 1.0;
        bm.scale.x = bm.scale.y = bm.scale.z = 0.2;
        bm.color.r = 0.0f;
        bm.color.g = 0.5f;
        bm.color.b = 1.0f;
        bm.color.a = 0.9f;
        bm.lifetime = rclcpp::Duration::from_seconds(2.0);
        ma.markers.push_back(bm);

        marker_pub_->publish(ma);
    }

    std::string map_frame_, robot_frame_;
    double sensor_radius_, goal_min_distance_, coverage_threshold_;
    double radius_initial_, radius_step_, radius_max_;
    int samples_per_cycle_;
    double corridor_bonus_, advisory_stale_secs_;

    mutable double sample_radius_;
    mutable std::mt19937 rng_{std::random_device{}()};

    std::mutex map_mutex_;
    bool map_dirty_ = false;
    nav_msgs::msg::OccupancyGrid::SharedPtr thermal_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr spatial_map_;
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr spatial_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr eval_timer_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    bool robot_pose_valid_ = true;
};

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::AdvisoryNode>());
    rclcpp::shutdown();
    return 0;
}
