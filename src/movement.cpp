#include <random>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>
#include <random>

using std::placeholders::_1;

// Disederhanakan buat continuous target-following (bukan one-shot goal):
// - ROTATE_TO_GOAL : putar di tempat sampai menghadap target
// - MOVE_TO_GOAL   : maju sambil terus koreksi heading (pure pursuit sederhana)
// - ARRIVED        : sudah cukup deket, robot diam tapi TETAP mantau -> begitu
//                    target menjauh lagi (lewat resume_tolerance), lanjut ngejar lagi
enum class MoveState { IDLE, ROTATE_TO_GOAL, MOVE_TO_GOAL, ARRIVED };

class MovementNode : public rclcpp::Node
{
public:
    MovementNode() : Node("movement_node")
    {
        declare_parameter("max_linear_speed", 0.3);
        declare_parameter("max_angular_speed", 1.0);
        declare_parameter("kp_linear", 0.6);
        declare_parameter("kp_angular", 1.8);
        declare_parameter("arrive_tolerance", 0.15);     // meter, jarak dianggap "sampai"/cukup deket target
        declare_parameter("resume_tolerance", 0.30);     // meter, jarak minimal sebelum mulai ngejar lagi (hysteresis)
        declare_parameter("yaw_tolerance", 0.15);        // rad (~8.5 deg)
        declare_parameter("rotate_start_threshold", 0.4);// rad, kalau heading error > ini, stop & putar dulu
        declare_parameter("control_hz", 20.0);

        max_lin_   = get_parameter("max_linear_speed").as_double();
        max_ang_   = get_parameter("max_angular_speed").as_double();
        kp_lin_    = get_parameter("kp_linear").as_double();
        kp_ang_    = get_parameter("kp_angular").as_double();
        arrive_tol_ = get_parameter("arrive_tolerance").as_double();
        resume_tol_ = get_parameter("resume_tolerance").as_double();
        yaw_tol_   = get_parameter("yaw_tolerance").as_double();
        rotate_thresh_ = get_parameter("rotate_start_threshold").as_double();

        double hz = get_parameter("control_hz").as_double();

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose", 10, std::bind(&MovementNode::poseCb, this, _1));

        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&MovementNode::goalCb, this, _1));

        cmd_pub_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        reached_pub_ = create_publisher<std_msgs::msg::Bool>("/goal_reached", 10);

        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / hz),
            std::bind(&MovementNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "movement_node siap. Nunggu /goal_pose...");
    }

private:
    // ---------------- callbacks ----------------
    void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        cur_x_ = msg->pose.position.x;
        cur_y_ = msg->pose.position.y;
        cur_yaw_ = yawFromQuat(msg->pose.orientation);
        has_pose_ = true;
    }

    void goalCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        has_goal_ = true;

        // PENTING: goal_pose datang terus-menerus (tiap frame kamera). Jangan paksa
        // reset state tiap kali, biar controlLoop yang jalan mulus tidak ke-interupsi.
        // Cukup mulai state machine kalau ini goal PERTAMA (masih IDLE).
        if (state_ == MoveState::IDLE) {
            state_ = MoveState::ROTATE_TO_GOAL;
            RCLCPP_INFO(get_logger(), "Target pertama diterima: x=%.2f y=%.2f", goal_x_, goal_y_);
        }
    }

    // ---------------- helper ----------------
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

    void publishCmd(double lin, double ang)
    {
        // Tambahin noise ±5% (sesuaikan dengan real life)
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-0.05, 0.05);
        
        double noisy_lin = lin * (1.0 + dis(gen));
        double noisy_ang = ang * (1.0 + dis(gen));
    
        geometry_msgs::msg::Twist cmd;
         cmd.linear.x = std::clamp(noisy_lin, -max_lin_, max_lin_);
        cmd.angular.z = std::clamp(noisy_ang, -max_ang_, max_ang_);
        cmd_pub_->publish(cmd);
    }

    void stopRobot()
    {
        publishCmd(0.0, 0.0);
    }

    // ---------------- main loop ----------------
    void controlLoop()
    {
        if (!has_pose_ || !has_goal_) return;
        if (state_ == MoveState::IDLE) return;

        double dx = goal_x_ - cur_x_;
        double dy = goal_y_ - cur_y_;
        double distance = std::hypot(dx, dy);
        double angle_to_goal = std::atan2(dy, dx);
        double heading_error = normalizeAngle(angle_to_goal - cur_yaw_);

        switch (state_)
        {
        case MoveState::ROTATE_TO_GOAL:
        {
            if (distance < arrive_tol_) {
                stopRobot();
                state_ = MoveState::ARRIVED;
                break;
            }
            if (std::fabs(heading_error) > yaw_tol_) {
                publishCmd(0.0, kp_ang_ * heading_error);
            } else {
                state_ = MoveState::MOVE_TO_GOAL;
                RCLCPP_INFO(get_logger(), "Heading oke, mulai ngejar target.");
            }
            break;
        }

        case MoveState::MOVE_TO_GOAL:
        {
            if (distance < arrive_tol_) {
                stopRobot();
                state_ = MoveState::ARRIVED;
                std_msgs::msg::Bool msg;
                msg.data = true;
                reached_pub_->publish(msg);
                RCLCPP_INFO(get_logger(), "Sampai di target (jarak %.2f m).", distance);
                break;
            }

            // heading error kebesaran (target belok tajam / berpindah drastis) -> stop & putar ulang
            if (std::fabs(heading_error) > rotate_thresh_) {
                stopRobot();
                state_ = MoveState::ROTATE_TO_GOAL;
                break;
            }

            double linear = kp_lin_ * distance * std::cos(heading_error);
            double angular = kp_ang_ * heading_error;
            publishCmd(linear, angular);
            break;
        }

        case MoveState::ARRIVED:
        {
            // Diam, tapi terus mantau. Pakai hysteresis (resume_tol_ > arrive_tol_)
            // biar gak gerak-diam-gerak-diam terus kalau target cuma geser dikit di sekitar batas.
            if (distance > resume_tol_) {
                std_msgs::msg::Bool msg;
                msg.data = false;
                reached_pub_->publish(msg);
                state_ = (std::fabs(heading_error) > rotate_thresh_)
                             ? MoveState::ROTATE_TO_GOAL
                             : MoveState::MOVE_TO_GOAL;
                RCLCPP_INFO(get_logger(), "Target menjauh lagi (%.2f m), lanjut ngejar.", distance);
            } else {
                stopRobot();
            }
            break;
        }

        default:
            break;
        }
    }

    // ---------------- members ----------------
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reached_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    MoveState state_ = MoveState::IDLE;
    bool has_pose_ = false;
    bool has_goal_ = false;

    double cur_x_ = 0, cur_y_ = 0, cur_yaw_ = 0;
    double goal_x_ = 0, goal_y_ = 0;

    double max_lin_, max_ang_, kp_lin_, kp_ang_, arrive_tol_, resume_tol_, yaw_tol_, rotate_thresh_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MovementNode>());
    rclcpp::shutdown();
    return 0;
}