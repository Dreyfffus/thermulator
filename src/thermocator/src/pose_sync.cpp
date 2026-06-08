
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_interfaces/msg/entity.hpp>
#include <ros_gz_interfaces/srv/set_entity_pose.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace thermocator {

class PoseSyncNode : public rclcpp::Node {
  public:
    PoseSyncNode() : Node("pose_sync_node") {
        declare_parameter("world_name", std::string("my_world"));
        declare_parameter("robot_entity_name", std::string("turtlebot3_burger"));
        declare_parameter("map_frame", std::string("map"));
        declare_parameter("robot_frame", std::string("base_footprint"));
        declare_parameter("sync_rate_hz", 1.0);
        declare_parameter("translation_deadband", 0.05);
        declare_parameter("rotation_deadband", 0.05);
        declare_parameter("robot_spawn_z", 0.01);

        world_name_ = get_parameter("world_name").as_string();
        robot_entity_name_ = get_parameter("robot_entity_name").as_string();
        map_frame_ = get_parameter("map_frame").as_string();
        robot_frame_ = get_parameter("robot_frame").as_string();
        sync_rate_hz_ = get_parameter("sync_rate_hz").as_double();
        translation_deadband_ = get_parameter("translation_deadband").as_double();
        rotation_deadband_ = get_parameter("rotation_deadband").as_double();
        robot_spawn_z_ = get_parameter("robot_spawn_z").as_double();

        set_pose_client_ = create_client<ros_gz_interfaces::srv::SetEntityPose>(
            "/world/" + world_name_ + "/set_pose");

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::QoS(10),
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                robot_x_ = msg->pose.pose.position.x;
                robot_y_ = msg->pose.pose.position.y;
                robot_qz_ = msg->pose.pose.orientation.z;
                robot_qw_ = msg->pose.pose.orientation.w;
                robot_pose_valid_ = true;
            });

        const auto period = std::chrono::duration<double>(1.0 / sync_rate_hz_);
        sync_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PoseSyncNode::syncPose, this));

        RCLCPP_INFO(get_logger(),
                    "PoseSyncNode ready: entity=%s world=%s rate=%.1fHz",
                    robot_entity_name_.c_str(),
                    world_name_.c_str(),
                    sync_rate_hz_);
    }

  private:
    void syncPose() {

        if (!service_ready_) {
            if (!set_pose_client_->service_is_ready()) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                     "PoseSyncNode: waiting for /world/%s/set_pose",
                                     world_name_.c_str());
                return;
            }
            RCLCPP_INFO(get_logger(),
                        "PoseSyncNode: /world/%s/set_pose ready",
                        world_name_.c_str());
            service_ready_ = true;
        }

        if (!robot_pose_valid_)
            return;

        const double tx = robot_x_;
        const double ty = robot_y_;
        const double qz = robot_qz_;
        const double qw = robot_qw_;

        const double dt = std::sqrt(
            std::pow(tx - last_tx_, 2) + std::pow(ty - last_ty_, 2));
        const double last_yaw = std::atan2(2.0 * last_qw_ * last_qz_,
                                           1.0 - 2.0 * last_qz_ * last_qz_);
        const double curr_yaw = std::atan2(2.0 * qw * qz,
                                           1.0 - 2.0 * qz * qz);
        const double dr = std::abs(curr_yaw - last_yaw);

        if (dt < translation_deadband_ && dr < rotation_deadband_)
            return;

        if (!set_pose_client_->wait_for_service(std::chrono::milliseconds(500))) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                 "PoseSyncNode: /world/%s/set_pose not available",
                                 world_name_.c_str());
            return;
        }

        auto req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        req->entity.name = robot_entity_name_;
        req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;

        req->pose.position.x = tx;
        req->pose.position.y = ty;
        req->pose.position.z = robot_spawn_z_;

        req->pose.orientation.x = 0.0;
        req->pose.orientation.y = 0.0;
        req->pose.orientation.z = qz;
        req->pose.orientation.w = qw;

        set_pose_client_->async_send_request(
            req,
            [this, tx, ty](
                rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedFuture f) {
                if (!f.get()->success)
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                                         "PoseSyncNode: set_pose rejected by Gazebo");
                else
                    RCLCPP_DEBUG(get_logger(),
                                 "PoseSyncNode: synced to (%.3f, %.3f)", tx, ty);
            });

        last_tx_ = tx;
        last_ty_ = ty;
        last_qz_ = qz;
        last_qw_ = qw;
    }

    std::string world_name_;
    std::string robot_entity_name_;
    std::string map_frame_;
    std::string robot_frame_;
    double sync_rate_hz_;
    double translation_deadband_;
    double rotation_deadband_;
    double robot_spawn_z_;

    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_qz_ = 0.0;
    double robot_qw_ = 1.0;
    bool robot_pose_valid_ = false;

    double last_tx_ = 1e9;
    double last_ty_ = 1e9;
    double last_qz_ = 0.0;
    double last_qw_ = 1.0;

    bool service_ready_ = false;

    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr set_pose_client_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
};

} // namespace thermocator

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<thermocator::PoseSyncNode>());
    rclcpp::shutdown();
    return 0;
}
