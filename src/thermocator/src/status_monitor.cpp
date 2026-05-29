#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/string.hpp>

namespace thermocator {

class StatusMonitor : public rclcpp::Node {
  public:
    StatusMonitor() : Node("status_monitor") {
        declare_parameter("obstacle_warning_distance", 0.45);
        declare_parameter("publish_rate", 2.0);
        declare_parameter("initial_battery_level", 100.0);
        declare_parameter("battery_drain_per_second", 0.02);

        obstacle_warning_distance_ =
            get_parameter("obstacle_warning_distance").as_double();
        battery_level_ = get_parameter("initial_battery_level").as_double();
        battery_drain_per_second_ =
            get_parameter("battery_drain_per_second").as_double();
        last_battery_update_ = now();

        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::QoS(10),
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                last_scan_time_ = now();
                scan_received_ = true;
                nearest_obstacle_m_ = nearestObstacle(*msg);
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                last_odom_time_ = now();
                odom_received_ = true;
                speed_mps_ = std::hypot(
                    msg->twist.twist.linear.x,
                    msg->twist.twist.linear.y);
            });

        thermal_sub_ = create_subscription<sensor_msgs::msg::Temperature>(
            "/thermal_reading", rclcpp::QoS(10),
            [this](sensor_msgs::msg::Temperature::SharedPtr msg) {
                last_thermal_time_ = now();
                thermal_received_ = true;
                last_thermal_value_ = msg->temperature;
            });

        status_pub_ = create_publisher<std_msgs::msg::String>("/robot/status", 10);
        environment_pub_ =
            create_publisher<std_msgs::msg::String>("/robot/environment_event", 10);

        const double rate = get_parameter("publish_rate").as_double();
        const auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&StatusMonitor::publishStatus, this));

        RCLCPP_INFO(get_logger(),
                    "Status monitor ready: publishing /robot/status and /robot/environment_event");
    }

  private:
    double nearestObstacle(const sensor_msgs::msg::LaserScan &scan) const {
        double nearest = std::numeric_limits<double>::infinity();
        for (const auto range : scan.ranges) {
            if (!std::isfinite(range)) {
                continue;
            }
            if (range < scan.range_min || range > scan.range_max) {
                continue;
            }
            nearest = std::min(nearest, static_cast<double>(range));
        }
        return nearest;
    }

    bool fresh(const rclcpp::Time &stamp, double max_age_seconds) const {
        if (stamp.nanoseconds() == 0) {
            return false;
        }
        return (now() - stamp).seconds() <= max_age_seconds;
    }

    std::string mode() const {
        if (!scan_received_ || !odom_received_) {
            return "STARTING";
        }
        if (obstacleDetected()) {
            return "OBSTACLE_RESPONSE";
        }
        if (speed_mps_ > 0.02) {
            return "NAVIGATING";
        }
        return "SCANNING";
    }

    bool obstacleDetected() const {
        return std::isfinite(nearest_obstacle_m_) &&
               nearest_obstacle_m_ <= obstacle_warning_distance_;
    }

    void publishStatus() {
        updateBattery();

        const bool scan_ok = scan_received_ && fresh(last_scan_time_, 2.0);
        const bool odom_ok = odom_received_ && fresh(last_odom_time_, 2.0);
        const bool thermal_ok = thermal_received_ && fresh(last_thermal_time_, 3.0);
        const bool obstacle_detected = obstacleDetected();

        std_msgs::msg::String status_msg;
        std::ostringstream status;
        status << "mode=" << mode()
               << "; battery_level=" << battery_level_
               << "; battery_ok=" << (battery_level_ > 20.0 ? "true" : "false")
               << "; scan_ok=" << (scan_ok ? "true" : "false")
               << "; odom_ok=" << (odom_ok ? "true" : "false")
               << "; thermal_ok=" << (thermal_ok ? "true" : "false")
               << "; obstacle_detected=" << (obstacle_detected ? "true" : "false")
               << "; nearest_obstacle_m=";
        if (std::isfinite(nearest_obstacle_m_)) {
            status << nearest_obstacle_m_;
        } else {
            status << "nan";
        }
        status << "; speed_mps=" << speed_mps_
               << "; thermal_value=";
        if (thermal_received_) {
            status << last_thermal_value_;
        } else {
            status << "nan";
        }
        status_msg.data = status.str();
        status_pub_->publish(status_msg);

        std_msgs::msg::String environment_msg;
        std::ostringstream environment;
        environment << "event="
                    << (obstacle_detected ? "OBSTACLE_NEARBY" : "CLEAR")
                    << "; nearest_obstacle_m=";
        if (std::isfinite(nearest_obstacle_m_)) {
            environment << nearest_obstacle_m_;
        } else {
            environment << "nan";
        }
        environment << "; mirrored_to_dt=true";
        environment_msg.data = environment.str();
        environment_pub_->publish(environment_msg);
    }

    void updateBattery() {
        const auto current_time = now();
        if (last_battery_update_.nanoseconds() == 0) {
            last_battery_update_ = current_time;
            return;
        }

        const double elapsed = (current_time - last_battery_update_).seconds();
        if (elapsed <= 0.0) {
            return;
        }

        battery_level_ = std::max(0.0, battery_level_ - elapsed * battery_drain_per_second_);
        last_battery_update_ = current_time;
    }

    double obstacle_warning_distance_{0.45};
    double nearest_obstacle_m_{std::numeric_limits<double>::infinity()};
    double speed_mps_{0.0};
    double last_thermal_value_{0.0};
    double battery_level_{100.0};
    double battery_drain_per_second_{0.02};
    bool scan_received_{false};
    bool odom_received_{false};
    bool thermal_received_{false};
    rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_thermal_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_battery_update_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr thermal_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr environment_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::StatusMonitor>());
    rclcpp::shutdown();
    return 0;
}
