#include <cmath>
#include <string>
#include <unordered_map>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

namespace thermocator {

struct StreamSample {
    rclcpp::Time header_stamp;
    rclcpp::Time arrival_stamp;
    bool has_header_stamp{false};
    bool received{false};
};

class SyncMonitor : public rclcpp::Node {
  public:
    SyncMonitor() : Node("sync_monitor") {
        declare_parameter("tolerance_seconds", 0.5);
        tolerance_seconds_ = get_parameter("tolerance_seconds").as_double();

        auto qos = rclcpp::QoS(10);
        auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                updateOriginal("odom", msg->header.stamp);
            });
        dt_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/dt/odom", qos,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                updateDt("odom", msg->header.stamp);
            });

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                updateOriginal("scan", msg->header.stamp);
            });
        dt_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/dt/scan", qos,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                updateDt("scan", msg->header.stamp);
            });

        thermal_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/thermal_map", latched_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                updateOriginal("thermal_map", msg->header.stamp);
            });
        dt_thermal_map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/dt/thermal_map", latched_qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                updateDt("thermal_map", msg->header.stamp);
            });

        robot_status_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot/status", qos,
            [this](std_msgs::msg::String::SharedPtr) {
                updateOriginalArrivalOnly("robot_status");
            });
        dt_robot_status_sub_ = create_subscription<std_msgs::msg::String>(
            "/dt/robot_status", qos,
            [this](std_msgs::msg::String::SharedPtr) {
                updateDtArrivalOnly("robot_status");
            });

        environment_event_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot/environment_event", qos,
            [this](std_msgs::msg::String::SharedPtr) {
                updateOriginalArrivalOnly("environment_event");
            });
        dt_environment_event_sub_ = create_subscription<std_msgs::msg::String>(
            "/dt/environment_event", qos,
            [this](std_msgs::msg::String::SharedPtr) {
                updateDtArrivalOnly("environment_event");
            });

        RCLCPP_INFO(
            get_logger(),
            "Sync monitor ready: tolerance %.3f seconds", tolerance_seconds_);
    }

  private:
    void updateOriginal(const std::string &name, const builtin_interfaces::msg::Time &stamp) {
        update(samples_original_[name], stamp);
        report(name);
    }

    void updateDt(const std::string &name, const builtin_interfaces::msg::Time &stamp) {
        update(samples_dt_[name], stamp);
        report(name);
    }

    void updateOriginalArrivalOnly(const std::string &name) {
        updateArrivalOnly(samples_original_[name]);
        report(name);
    }

    void updateDtArrivalOnly(const std::string &name) {
        updateArrivalOnly(samples_dt_[name]);
        report(name);
    }

    void update(StreamSample &sample, const builtin_interfaces::msg::Time &stamp) {
        sample.arrival_stamp = now();
        sample.header_stamp = rclcpp::Time(stamp);
        sample.has_header_stamp = stamp.sec != 0 || stamp.nanosec != 0;
        sample.received = true;
    }

    void updateArrivalOnly(StreamSample &sample) {
        sample.arrival_stamp = now();
        sample.header_stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
        sample.has_header_stamp = false;
        sample.received = true;
    }

    void report(const std::string &name) {
        const auto original_it = samples_original_.find(name);
        const auto dt_it = samples_dt_.find(name);
        if (original_it == samples_original_.end() || dt_it == samples_dt_.end()) {
            return;
        }

        const auto &original = original_it->second;
        const auto &dt = dt_it->second;
        if (!original.received || !dt.received) {
            return;
        }

        const double arrival_delta =
            std::abs((dt.arrival_stamp - original.arrival_stamp).seconds());

        if (original.has_header_stamp && dt.has_header_stamp) {
            const double header_delta =
                std::abs((dt.header_stamp - original.header_stamp).seconds());
            logResult(name, header_delta, arrival_delta, true);
        } else {
            logResult(name, arrival_delta, arrival_delta, false);
        }
    }

    void logResult(
        const std::string &name,
        double primary_delta,
        double arrival_delta,
        bool used_header_stamp) {
        const char *source = used_header_stamp ? "header" : "arrival";

        if (primary_delta > tolerance_seconds_) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Sync warning [%s]: %s delta %.3fs exceeds tolerance %.3fs; arrival skew %.3fs",
                name.c_str(), source, primary_delta, tolerance_seconds_, arrival_delta);
            return;
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Sync ok [%s]: %s delta %.3fs; arrival skew %.3fs",
            name.c_str(), source, primary_delta, arrival_delta);
    }

    double tolerance_seconds_{0.5};
    std::unordered_map<std::string, StreamSample> samples_original_;
    std::unordered_map<std::string, StreamSample> samples_dt_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dt_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr dt_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr thermal_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dt_thermal_map_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_robot_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr environment_event_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dt_environment_event_sub_;
};

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::SyncMonitor>());
    rclcpp::shutdown();
    return 0;
}
