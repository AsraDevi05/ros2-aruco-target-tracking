#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cmath>
#include <deque>

class TargetBridge : public rclcpp::Node
{
public:
  TargetBridge() : Node("target_bridge")
  {
    declare_parameter("publish_hz",    20.0);
    declare_parameter("smooth_window", 5);
    declare_parameter("min_move_dist", 0.05);

    publish_hz_    = get_parameter("publish_hz").as_double();
    smooth_window_ = get_parameter("smooth_window").as_int();
    min_move_dist_ = get_parameter("min_move_dist").as_double();

    sub_target_ = create_subscription<geometry_msgs::msg::Point>(
        "/target_position", 10,
        std::bind(&TargetBridge::targetCb, this, std::placeholders::_1));

    sub_aruco_ = create_subscription<geometry_msgs::msg::Point>(
        "/aruco_position", 10,
        std::bind(&TargetBridge::arucoCb, this, std::placeholders::_1));

    sub_reached_ = create_subscription<std_msgs::msg::Bool>(
        "/goal_reached", 10,
        std::bind(&TargetBridge::reachedCb, this, std::placeholders::_1));

    pub_goal_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_hz_),
        std::bind(&TargetBridge::publishGoal, this));

    RCLCPP_INFO(get_logger(), "target_bridge ready — jembatan detector->movement");
  }

private:
  void targetCb(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    buf_.push_back({msg->x, msg->y});
    if ((int)buf_.size() > smooth_window_) buf_.pop_front();

    double sx = 0, sy = 0;
    for (auto &p : buf_) { sx += p.first; sy += p.second; }
    double nx = sx / buf_.size(), ny = sy / buf_.size();

    if (std::hypot(nx - last_x_, ny - last_y_) > min_move_dist_ || !has_target_) {
      target_x_ = nx; target_y_ = ny;
      last_x_ = nx;   last_y_ = ny;
      has_target_ = true;
    }
  }

  void arucoCb(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    aruco_x_ = msg->x; aruco_y_ = msg->y;
    has_aruco_ = true;
  }

  void reachedCb(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Goal reached!");
  }

  void publishGoal()
  {
    if (!has_target_) return;

    geometry_msgs::msg::PoseStamped goal;
    goal.header.stamp    = now();
    goal.header.frame_id = "map";
    goal.pose.position.x = target_x_;
    goal.pose.position.y = target_y_;
    goal.pose.position.z = 0.0;

    // Orientasi: hadap ke arah bola dari posisi ArUco
    if (has_aruco_) {
      double yaw = std::atan2(target_y_ - aruco_y_, target_x_ - aruco_x_);
      goal.pose.orientation.z = std::sin(yaw / 2.0);
      goal.pose.orientation.w = std::cos(yaw / 2.0);
    } else {
      goal.pose.orientation.w = 1.0;
    }

    pub_goal_->publish(goal);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        "goal_pose -> (%.2f, %.2f)", target_x_, target_y_);
  }

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr    sub_target_, sub_aruco_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          sub_reached_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool   has_target_ = false, has_aruco_ = false;
  double target_x_ = 0, target_y_ = 0;
  double aruco_x_  = 0, aruco_y_  = 0;
  double last_x_   = 0, last_y_   = 0;
  double publish_hz_; int smooth_window_; double min_move_dist_;
  std::deque<std::pair<double,double>> buf_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetBridge>());
  rclcpp::shutdown();
  return 0;
}
