#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

// ── Tuning ────────────────────────────────────────────
static constexpr double LINEAR_SPEED      = 1.3;   // m/s
static constexpr double ANGULAR_SPEED     = 0.5;   // rad/s
static constexpr double WALL_DIST_TARGET  = 0.5;   // m, target jarak ke dinding kanan
static constexpr double WALL_DIST_TOL     = 0.08;  // m, toleransi koreksi
static constexpr double FRONT_DANGER_DIST = 0.5;   // m, langsung belok
static constexpr double FRONT_WARN_DIST   = 0.8;   // m, mulai lambat
// ──────────────────────────────────────────────────────

using LaserScan = sensor_msgs::msg::LaserScan;
using Twist     = geometry_msgs::msg::Twist;

static float getRange(const LaserScan::SharedPtr msg)
{
  if (!msg || msg->ranges.empty()) return 9.9f;
  float r = msg->ranges[0];
  if (std::isinf(r) || std::isnan(r) || r < msg->range_min) return 9.9f;
  return (r > msg->range_max) ? msg->range_max : r;
}

class WallFollower : public rclcpp::Node
{
public:
  WallFollower() : Node("wall_follower")
  {
    auto qos = rclcpp::SensorDataQoS();
    auto sub = [&](const std::string & topic, float & dst) {
      return create_subscription<LaserScan>(topic, qos,
        [&dst](LaserScan::SharedPtr m){ dst = getRange(m); });
    };

    sub_front_       = sub("/ultrasonic/front",       dist_front_);
    sub_front_right_ = sub("/ultrasonic/front_right", dist_front_right_);
    sub_right_       = sub("/ultrasonic/right",       dist_right_);
    sub_front_left_  = sub("/ultrasonic/front_left",  dist_front_left_);
    sub_left_        = sub("/ultrasonic/left",        dist_left_);
    sub_back_        = sub("/ultrasonic/back",        dist_back_);

    pub_ = create_publisher<Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
               std::bind(&WallFollower::loop, this));

    RCLCPP_INFO(get_logger(), "WallFollower started");
  }

private:
  float dist_front_ = 9.9f, dist_front_right_ = 9.9f, dist_front_left_ = 9.9f;
  float dist_right_ = 9.9f, dist_left_ = 9.9f, dist_back_ = 9.9f;

  rclcpp::Subscription<LaserScan>::SharedPtr
    sub_front_, sub_front_right_, sub_right_,
    sub_front_left_, sub_left_, sub_back_;
  rclcpp::Publisher<Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void loop()
  {
    Twist cmd;

    // 1. Bahaya depan → belok kiri
    if (dist_front_ < FRONT_DANGER_DIST || dist_front_right_ < FRONT_DANGER_DIST * 0.7) {
      cmd.angular.z = ANGULAR_SPEED;
      pub_->publish(cmd);
      return;
    }

    // 2. Tidak ada dinding kanan → cari dinding
    if (dist_right_ > WALL_DIST_TARGET + 0.5) {
      cmd.linear.x  = LINEAR_SPEED * 0.6;
      cmd.angular.z = -ANGULAR_SPEED * 0.5;
      pub_->publish(cmd);
      return;
    }

    // 3. Ikut dinding kanan
    double err = dist_right_ - WALL_DIST_TARGET;
    double angular = 0.0;
    if      (err >  WALL_DIST_TOL)              angular = -ANGULAR_SPEED * 0.4;
    else if (err < -WALL_DIST_TOL)              angular =  ANGULAR_SPEED * 0.4;
    if (dist_front_right_ < WALL_DIST_TARGET * 1.2) angular += ANGULAR_SPEED * 0.3;

    double speed = LINEAR_SPEED;
    if (dist_front_ < FRONT_WARN_DIST) speed *= dist_front_ / FRONT_WARN_DIST;

    cmd.linear.x  = speed;
    cmd.angular.z = angular;
    pub_->publish(cmd);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollower>());
  rclcpp::shutdown();
}