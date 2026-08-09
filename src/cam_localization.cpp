#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class PoseFusion : public rclcpp::Node
{
public:
    PoseFusion() : Node("pose_fusion"),
        x_(0.0), y_(0.0), yaw_(0.0),
        has_pos_(false), has_imu_(false)
    {
        sub_pos_ = create_subscription<geometry_msgs::msg::Point>(
            "/robot_position", 10,
            std::bind(&PoseFusion::posCb, this, std::placeholders::_1));

        sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&PoseFusion::imuCb, this, std::placeholders::_1));

        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10);

        RCLCPP_INFO(get_logger(), "Pose Fusion node started");
    }

private:
    void posCb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        x_ = msg->x;
        y_ = msg->y;
        has_pos_ = true;
        tryPublish();
    }

    void imuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Ekstrak yaw dari quaternion IMU
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        yaw_ = yaw;
        has_imu_ = true;
        tryPublish();
    }

    void tryPublish()
    {
        if (!has_pos_ || !has_imu_) return;

        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = now();
        pose.header.frame_id = "map";

        // Posisi dari kamera
        pose.pose.position.x = x_;
        pose.pose.position.y = y_;
        pose.pose.position.z = 0.0;

        // Orientasi dari IMU
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pub_pose_->publish(pose);

        RCLCPP_INFO(get_logger(), "Pose -> x: %.2f  y: %.2f  yaw: %.2f deg",
            x_, y_, yaw_ * 180.0 / M_PI);
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pos_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

    double x_, y_, yaw_;
    bool has_pos_, has_imu_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFusion>());
    rclcpp::shutdown();
    return 0;
}