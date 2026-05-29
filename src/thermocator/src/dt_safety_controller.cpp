#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace thermocator {

class DtSafetyController : public rclcpp::Node {
  public:
    DtSafetyController() : Node("dt_safety_controller") {
        declare_parameter("publish_rate", 5.0);
        declare_parameter("hold_seconds_after_obstacle", 1.0);

        hold_seconds_after_obstacle_ =
            get_parameter("hold_seconds_after_obstacle").as_double();

        environment_sub_ = create_subscription<std_msgs::msg::String>(
            "/dt/environment_event", rclcpp::QoS(10),
            [this](std_msgs::msg::String::SharedPtr msg) {
                if (msg->data.find("event=OBSTACLE_NEARBY") != std::string::npos) {
                    obstacle_active_ = true;
                    last_obstacle_time_ = now();
                    RCLCPP_WARN_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "DT safety controller observed mirrored obstacle event; publishing stop command");
                    return;
                }

                if (msg->data.find("event=CLEAR") != std::string::npos) {
                    obstacle_active_ = false;
                }
            });

        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/dt/cmd_vel", 10);
        control_status_pub_ =
            create_publisher<std_msgs::msg::String>("/dt/control_status", 10);

        const double rate = get_parameter("publish_rate").as_double();
        const auto period = std::chrono::duration<double>(1.0 / rate);
        timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&DtSafetyController::controlLoop, this));

        RCLCPP_INFO(get_logger(),
                    "DT safety controller ready: /dt/environment_event -> /dt/cmd_vel stop commands");
    }

  private:
    bool holdActive() {
        if (last_obstacle_time_.nanoseconds() == 0) {
            return false;
        }
        return (now() - last_obstacle_time_).seconds() <= hold_seconds_after_obstacle_;
    }

    void controlLoop() {
        const bool should_stop = obstacle_active_ || holdActive();

        std_msgs::msg::String status;
        status.data = should_stop
                          ? "mode=SAFETY_STOP; source=/dt/environment_event; command=/dt/cmd_vel"
                          : "mode=MONITORING; source=/dt/environment_event";
        control_status_pub_->publish(status);

        if (!should_stop) {
            return;
        }

        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.linear.y = 0.0;
        stop.linear.z = 0.0;
        stop.angular.x = 0.0;
        stop.angular.y = 0.0;
        stop.angular.z = 0.0;
        cmd_pub_->publish(stop);
    }

    bool obstacle_active_{false};
    double hold_seconds_after_obstacle_{1.0};
    rclcpp::Time last_obstacle_time_{0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr environment_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::DtSafetyController>());
    rclcpp::shutdown();
    return 0;
}
