// =============================================================================
//  goal_arbiter.cpp
//
//  Runs on Domain 38. Collects scored goal candidates from every decision node
//  on /thermocator/goals:
//    - LOCAL   candidates from the Domain 38 decision node (published directly)
//    - TWINNED candidates from the Domain 1 decision node (bridged 1 -> 38)
//
//  Each control cycle it discards stale candidates, picks the highest-scoring
//  fresh one, sends it to Nav2 (navigate_to_pose) and draws a labelled marker
//  (LOCAL / TWINNED) on the RViz instance so the operator can see which part of
//  the system produced the goal currently being executed.
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "thermocator_msgs/msg/goal_candidate.hpp"

namespace thermocator
{

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class GoalArbiter : public rclcpp::Node {
public:
  GoalArbiter()
  : Node("goal_arbiter")
  {
    map_frame_ = declare_parameter("map_frame", std::string("map"));
    candidate_stale_secs_ = declare_parameter("candidate_stale_secs", 5.0);
    goal_timeout_seconds_ = declare_parameter("goal_timeout_seconds", 70.0);
    resend_min_distance_ = declare_parameter("resend_min_distance", 0.5);
    resend_score_margin_ = declare_parameter("resend_score_margin", 1.0);
    const double control_rate = declare_parameter("control_rate", 1.0);

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

    goal_sub_ = create_subscription<thermocator_msgs::msg::GoalCandidate>(
            "/thermocator/goals", rclcpp::QoS(10).reliable(),
            std::bind(&GoalArbiter::onCandidate, this, std::placeholders::_1));

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
            "/thermocator/goal_markers",
            rclcpp::QoS(1).transient_local().reliable());

    send_goal_options_.goal_response_callback =
      std::bind(&GoalArbiter::goalResponse, this, std::placeholders::_1);
    send_goal_options_.result_callback =
      std::bind(&GoalArbiter::goalResult, this, std::placeholders::_1);

    const auto period = std::chrono::duration<double>(1.0 / control_rate);
    timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&GoalArbiter::tick, this));

    sent_time_ = now();

    RCLCPP_INFO(get_logger(),
                    "GoalArbiter ready -- selecting best of /thermocator/goals by score");
  }

private:
  struct Entry
  {
    thermocator_msgs::msg::GoalCandidate cand;
    rclcpp::Time received;
  };

  void onCandidate(const thermocator_msgs::msg::GoalCandidate::SharedPtr msg)
  {
    const std::string key = msg->source.empty() ? std::string("LOCAL") : msg->source;
    latest_[key] = Entry{*msg, now()};
  }

  void tick()
  {
    if (!nav_client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "Waiting for Nav2 action server ...");
      return;
    }

    const rclcpp::Time t = now();
    const Entry *best = nullptr;
    for (auto it = latest_.begin(); it != latest_.end(); ) {
      const double age = (t - it->second.received).seconds();
      if (age > candidate_stale_secs_) {
        it = latest_.erase(it);
        continue;
      }
      if (!best || it->second.cand.score > best->cand.score) {
        best = &it->second;
      }
      ++it;
    }
    if (!best) {
      return;
    }

    const double bx = best->cand.pose.pose.position.x;
    const double by = best->cand.pose.pose.position.y;
    const double bs = best->cand.score;
    const std::string src = best->cand.source.empty() ? std::string("LOCAL") :
      best->cand.source;

    if (goal_active_) {
      const double dx = bx - current_x_;
      const double dy = by - current_y_;
      const bool moved = std::sqrt(dx * dx + dy * dy) > resend_min_distance_;
      const bool better = bs > current_score_ + resend_score_margin_;
      const double elapsed = (now() - sent_time_).seconds();
      const bool timed_out = elapsed > goal_timeout_seconds_;
      if (!moved && !better && !timed_out) {
        return;
      }
    }

    sendGoal(bx, by, bs, src);
  }

  void sendGoal(double x, double y, double score, const std::string & src)
  {
    NavigateToPose::Goal goal;
    goal.pose.header.stamp = now();
    goal.pose.header.frame_id = map_frame_;
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;
    nav_client_->async_send_goal(goal, send_goal_options_);

    current_x_ = x;
    current_y_ = y;
    current_score_ = score;
    current_src_ = src;
    goal_active_ = true;
    sent_time_ = now();

    publishMarker(x, y, src);
    RCLCPP_INFO(get_logger(), "[Arbiter] -> (%.2f,%.2f) score=%.1f src=%s",
                    x, y, score, src.c_str());
  }

  void goalResponse(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr & h)
  {
    if (!h) {
      goal_active_ = false;
      RCLCPP_WARN(get_logger(), "[Arbiter] goal rejected by Nav2");
    }
  }

  void goalResult(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & r)
  {
    goal_active_ = false;
    if (r.code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "[Arbiter] goal reached (%s)",
                        current_src_.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "[Arbiter] goal ended without success");
    }
  }

  void publishMarker(double x, double y, const std::string & src)
  {
    visualization_msgs::msg::MarkerArray ma;

    visualization_msgs::msg::Marker m;
    m.header.stamp = now();
    m.header.frame_id = map_frame_;
    m.ns = "arbiter_goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = x;
    m.pose.position.y = y;
    m.pose.position.z = 0.15;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 0.22;
    if (src == "TWINNED") {
      m.color.r = 0.0f;
      m.color.g = 0.5f;
      m.color.b = 1.0f;
    } else {
      m.color.r = 0.4f;
      m.color.g = 1.0f;
      m.color.b = 0.0f;
    }
    m.color.a = 0.9f;
    m.lifetime = rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(m);

    visualization_msgs::msg::Marker txt;
    txt.header = m.header;
    txt.ns = "arbiter_goal_source";
    txt.id = 0;
    txt.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::msg::Marker::ADD;
    txt.pose.position.x = x;
    txt.pose.position.y = y;
    txt.pose.position.z = 0.35;
    txt.pose.orientation.w = 1.0;
    txt.scale.z = 0.12;
    txt.color.r = txt.color.g = txt.color.b = 1.0f;
    txt.color.a = 1.0f;
    txt.text = src;
    txt.lifetime = rclcpp::Duration::from_seconds(0);
    ma.markers.push_back(txt);

    marker_pub_->publish(ma);
  }

  std::string map_frame_;
  double candidate_stale_secs_;
  double goal_timeout_seconds_;
  double resend_min_distance_;
  double resend_score_margin_;

  std::map<std::string, Entry> latest_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<NavigateToPose>::SendGoalOptions send_goal_options_;
  rclcpp::Subscription<thermocator_msgs::msg::GoalCandidate>::SharedPtr goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool goal_active_ = false;
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_score_ = 0.0;
  std::string current_src_;
  rclcpp::Time sent_time_;
};

} // namespace thermocator

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thermocator::GoalArbiter>());
  rclcpp::shutdown();
  return 0;
}
