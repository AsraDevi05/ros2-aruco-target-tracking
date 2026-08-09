#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

using std::placeholders::_1;

// ============================================================================
// pose_fusion_node
//
// Gabungin dua sumber posisi:
//  - /aruco_pose  : ground truth dari kamera overhead, TAPI cuma update saat
//                    marker kedetect (bisa gap kalau marker ke-occlude,
//                    atau rate-nya lebih rendah dari odom)
//  - /odom        : dead-reckoning dari Gazebo, rate tinggi & selalu update,
//                    TAPI drift lama-lama (terutama yaw)
//
// Strategi (complementary filter sederhana, bukan full EKF -- cukup buat
// kebutuhan tracking target di ujian ini):
//  1. Tiap kali /aruco_pose masuk -> hitung transform (rotasi + translasi)
//     dari frame odom ke frame world, pakai odom pose PADA SAAT ITU sebagai
//     anchor. Pose fusi langsung di-snap ke pose ArUco (paling akurat).
//  2. Tiap kali /odom masuk (rate tinggi) -> transformasikan odom pose
//     pakai transform terakhir dari langkah 1, publish ke /pose.
//     Jadi selama gap antar-deteksi ArUco, /pose tetap update mulus &
//     kontinu ngikutin gerakan odom, cuma "digeser" sesuai anchor terakhir.
//
// Kalau /aruco_pose lama gak update (marker ke-occlude lama), pose fusi
// akan makin drift ngikutin akurasi odom -- ini expected & wajar untuk
// complementary filter simpel kayak gini.
// ============================================================================

class PoseFusion : public rclcpp::Node
{
public:
  PoseFusion() : Node("pose_fusion_node")
  {
    aruco_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/aruco_pose", 10, std::bind(&PoseFusion::arucoCb, this, _1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&PoseFusion::odomCb, this, _1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);

    RCLCPP_INFO(get_logger(), "pose_fusion_node started, menunggu /aruco_pose & /odom ...");
  }

private:
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

  void arucoCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!has_odom_) return; // butuh odom dulu buat jadi anchor

    // pose ArUco = ground truth
    double xa = msg->pose.position.x;
    double ya = msg->pose.position.y;
    double yawa = yawFromQuat(msg->pose.orientation);

    // odom pose saat ini = anchor
    double xo = last_odom_x_;
    double yo = last_odom_y_;
    double yawo = last_odom_yaw_;

    // hitung transform odom-frame -> world-frame
    // world = R(yaw_offset) * odom + translation
    yaw_offset_ = normalizeAngle(yawa - yawo);
    double c = std::cos(yaw_offset_);
    double s = std::sin(yaw_offset_);
    trans_x_ = xa - (c * xo - s * yo);
    trans_y_ = ya - (s * xo + c * yo);

    has_anchor_ = true;

    // langsung publish pose ArUco (paling akurat, snap ke sini)
    publishPose(msg->header.stamp, xa, ya, yawa);
  }

  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_x_ = msg->pose.pose.position.x;
    last_odom_y_ = msg->pose.pose.position.y;
    last_odom_yaw_ = yawFromQuat(msg->pose.pose.orientation);
    has_odom_ = true;

    if (!has_anchor_) return; // belum pernah dapat ArUco sama sekali, gak tau transform-nya

    // transformasikan odom pose sekarang -> world frame pakai anchor terakhir
    double c = std::cos(yaw_offset_);
    double s = std::sin(yaw_offset_);
    double x_world = trans_x_ + (c * last_odom_x_ - s * last_odom_y_);
    double y_world = trans_y_ + (s * last_odom_x_ + c * last_odom_y_);
    double yaw_world = normalizeAngle(last_odom_yaw_ + yaw_offset_);

    publishPose(msg->header.stamp, x_world, y_world, yaw_world);
  }

  void publishPose(const rclcpp::Time &stamp, double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = "world";
    out.pose.position.x = x;
    out.pose.position.y = y;
    out.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    out.pose.orientation = tf2::toMsg(q);
    pose_pub_->publish(out);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  bool has_odom_ = false;
  bool has_anchor_ = false;

  double last_odom_x_ = 0, last_odom_y_ = 0, last_odom_yaw_ = 0;
  double trans_x_ = 0, trans_y_ = 0, yaw_offset_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseFusion>());
  rclcpp::shutdown();
  return 0;
}