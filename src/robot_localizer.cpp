#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

using std::placeholders::_1;

// ============================================================================
// RobotLocalizer — Fusion ArUco + Odom
//
// ArUco (kamera overhead) = koreksi absolut posisi/orientasi
// Odom (Gazebo) = propagasi kontinu, dead-reckoning saat ArUco hilang
// ============================================================================

class RobotLocalizer : public rclcpp::Node
{
public:
    RobotLocalizer() : Node("robot_localizer")
    {
        // ── Parameter ─────────────────────────────────────────────────────
        declare_parameter("aruco_timeout_sec", 2.0);
        declare_parameter("field_size", 6.0);
        declare_parameter("field_offset", 3.0);

        aruco_timeout_ = get_parameter("aruco_timeout_sec").as_double();
        field_size_    = get_parameter("field_size").as_double();
        field_offset_  = get_parameter("field_offset").as_double();

        // ── Subscribers ───────────────────────────────────────────────────
        sub_aruco_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_pose", 10, std::bind(&RobotLocalizer::arucoCb, this, _1));

        sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RobotLocalizer::odomCb, this, _1));

        // ── Publishers ────────────────────────────────────────────────────
        pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);

        last_aruco_time_ = now();

        RCLCPP_INFO(get_logger(), 
            "Robot Localizer started | field_size=%.1f field_offset=%.1f timeout=%.1fs",
            field_size_, field_offset_, aruco_timeout_);
    }

private:
    // ── Utility ─────────────────────────────────────────────────────────
    static double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
    {
        tf2::Quaternion tq(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tq).getRPY(roll, pitch, yaw);
        return yaw;
    }

    static double normalizeAngle(double a)
    {
        return std::atan2(std::sin(a), std::cos(a));
    }

    // ── Callback ArUco ────────────────────────────────────────────────────
    void arucoCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        double aruco_x   = msg->pose.position.x;
        double aruco_y   = msg->pose.position.y;
        double aruco_yaw = yawFromQuat(msg->pose.orientation);

        if (!has_odom_) {
            // Belum ada odom → offset langsung = aruco
            offset_yaw_ = aruco_yaw;
            offset_x_   = aruco_x;
            offset_y_   = aruco_y;
            has_offset_ = true;
            last_aruco_time_ = now();
            publishFusedPose(aruco_x, aruco_y, aruco_yaw, msg->header.stamp);
            return;
        }

        // Hitung offset dari selisih ArUco vs Odom terakhir
        offset_yaw_ = normalizeAngle(aruco_yaw - last_odom_yaw_);

        double c = std::cos(offset_yaw_);
        double s = std::sin(offset_yaw_);

        offset_x_ = aruco_x - (c * last_odom_x_ - s * last_odom_y_);
        offset_y_ = aruco_y - (s * last_odom_x_ + c * last_odom_y_);

        has_offset_ = true;
        last_aruco_time_ = now();

        // Publish pakai ArUco (paling akurat)
        publishFusedPose(aruco_x, aruco_y, aruco_yaw, msg->header.stamp);
    }

    // ── Callback Odom ───────────────────────────────────────────────────
    void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_odom_x_   = msg->pose.pose.position.x;
        last_odom_y_   = msg->pose.pose.position.y;
        last_odom_yaw_ = yawFromQuat(msg->pose.pose.orientation);
        has_odom_ = true;

        if (!has_offset_) {
            // Belum pernah dapat ArUco → anggap odom = world
            publishFusedPose(last_odom_x_, last_odom_y_, last_odom_yaw_, msg->header.stamp);
            return;
        }

        // Transform odom → world pakai offset
        double c = std::cos(offset_yaw_);
        double s = std::sin(offset_yaw_);

        double world_x   = c * last_odom_x_ - s * last_odom_y_ + offset_x_;
        double world_y   = s * last_odom_x_ + c * last_odom_y_ + offset_y_;
        double world_yaw = normalizeAngle(last_odom_yaw_ + offset_yaw_);

        publishFusedPose(world_x, world_y, world_yaw, msg->header.stamp);

        // Warning kalau ArUco lama nggak update
        double dt = (now() - last_aruco_time_).seconds();
        if (dt > aruco_timeout_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                "ArUco timeout %.1fs — dead-reckoning only", dt);
        }
    }

    // ── Publish Pose ──────────────────────────────────────────────────────
    void publishFusedPose(double x, double y, double yaw, const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp    = stamp;
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();

        pub_pose_->publish(pose);
    }

    // ── Members ─────────────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_aruco_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;

    bool has_odom_   = false;
    bool has_offset_ = false;

    double last_odom_x_   = 0.0;
    double last_odom_y_   = 0.0;
    double last_odom_yaw_ = 0.0;

    double offset_x_   = 0.0;
    double offset_y_   = 0.0;
    double offset_yaw_ = 0.0;

    double field_size_    = 6.0;   // [NEW] sinkron dengan target_detector
    double field_offset_  = 3.0;   // [NEW] sinkron dengan target_detector
    double aruco_timeout_ = 2.0;

    rclcpp::Time last_aruco_time_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotLocalizer>());
    rclcpp::shutdown();
    return 0;
}