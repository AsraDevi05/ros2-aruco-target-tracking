#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class WaypointNav : public rclcpp::Node
{
public:
    WaypointNav() : Node("waypoint_nav"),
        current_x_(0.0), current_y_(0.0), current_yaw_(0.0),
        target_x_(0.0), target_y_(0.0),
        has_pose_(false), has_target_(false)
    {
        sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10,
            std::bind(&WaypointNav::poseCb, this, std::placeholders::_1));

        sub_target_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&WaypointNav::targetCb, this, std::placeholders::_1));

        pub_cmd_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        RCLCPP_INFO(get_logger(), "Object Follower started, waiting for target...");
    }

private:
    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_x_ = msg->pose.position.x;
        current_y_ = msg->pose.position.y;

        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_yaw_ = yaw;
        has_pose_ = true;
        navigate();
    }

    void targetCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_x_ = msg->pose.position.x;
        target_y_ = msg->pose.position.y;
        has_target_ = true;
    }

    void navigate()
    {
        if (!has_pose_ || !has_target_) {
            stop();
            return;
        }

        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double dist = std::hypot(dx, dy);

        // Udah deket target, stop
        if (dist < dist_tol_) {
            stop();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "At target (%.2f, %.2f)", target_x_, target_y_);
            return;
        }

        double target_yaw = std::atan2(dy, dx);
        double yaw_err = target_yaw - current_yaw_;

        while (yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
        while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

        geometry_msgs::msg::Twist cmd;

        if (std::fabs(yaw_err) > yaw_tol_) {
            cmd.linear.x  = 0.0;
            cmd.angular.z = std::clamp(kp_yaw_ * yaw_err, -max_ang_, max_ang_);
        } else {
            cmd.linear.x  = std::clamp(kp_lin_ * dist, 0.0, max_lin_);
            cmd.angular.z = std::clamp(kp_yaw_ * yaw_err, -max_ang_, max_ang_);
        }

        pub_cmd_->publish(cmd);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Target(%.2f,%.2f) Robot(%.2f,%.2f) dist=%.2f yaw_err=%.1f",
            target_x_, target_y_, current_x_, current_y_,
            dist, yaw_err * 180.0 / M_PI);
    }

    void stop()
    {
        geometry_msgs::msg::Twist cmd;
        pub_cmd_->publish(cmd);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

    double current_x_, current_y_, current_yaw_;
    double target_x_, target_y_;
    bool has_pose_, has_target_;

    const double kp_lin_   = 0.5;
    const double kp_yaw_   = 1.0;
    const double max_lin_  = 0.3;
    const double max_ang_  = 0.8;
    const double dist_tol_ = 0.15;
    const double yaw_tol_  = 0.1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNav>());
    rclcpp::shutdown();
    return 0;
}
