// =============================================================================
//  battery_monitor.cpp
//
//  Part of the "state synced" digital-twin features. The robot's battery state
//  is bridged from Domain 38 to Domain 1 by domain_bridge; this node (run on the
//  twin, Domain 1) subscribes to it and logs the latest level to the terminal
//  every 10 seconds.
// =============================================================================

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace thermocator
{

class BatteryMonitor : public rclcpp::Node {
public:
  BatteryMonitor()
  : Node("battery_monitor")
  {
    topic_ = declare_parameter("battery_topic", std::string("/battery_state"));
    log_period_ = declare_parameter("log_period_seconds", 10.0);

    sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            topic_, rclcpp::QoS(10),
      [this](sensor_msgs::msg::BatteryState::SharedPtr msg) {
        last_ = msg;
            });

    timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(log_period_)),
            std::bind(&BatteryMonitor::report, this));

    RCLCPP_INFO(get_logger(),
                    "BatteryMonitor ready -- logging %s every %.0fs",
                    topic_.c_str(), log_period_);
  }

private:
  void report()
  {
    if (!last_) {
      RCLCPP_INFO(get_logger(),
                        "[Battery] no reading received yet on %s", topic_.c_str());
      return;
    }
        // REP-147: percentage is 0..1. Some drivers report 0..100 instead.
    const float pct = last_->percentage;
    RCLCPP_INFO(get_logger(),
                    "[Battery] %.1f%%  (%.2f V, %.2f A)",
                    pct <= 1.0f ? pct * 100.0f : pct,
                    last_->voltage, last_->current);
  }

  std::string topic_;
  double log_period_ = 10.0;
  sensor_msgs::msg::BatteryState::SharedPtr last_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace thermocator

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thermocator::BatteryMonitor>());
  rclcpp::shutdown();
  return 0;
}
