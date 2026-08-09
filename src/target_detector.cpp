// ============================================================================
// target_detector_node.cpp  —  VERSI REAL LIFE (tanpa LIDAR)
//
// PERUBAHAN:
//   [+] Deteksi obstacle: semua warna SELAIN kuning/hijau/putih = obstacle
//   [+] Obstacle avoidance: kurangi linear speed + belok menghindar
//   [-] Hapus semua LIDAR/scan
//   [-] Hapus deteksi warna orange spesifik
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

class TargetDetector : public rclcpp::Node
{
public:
  TargetDetector() : Node("target_detector")
  {
    // ── Parameter HSV bola kuning ──────────────────────────────────────────
    declare_parameter("field_size",   6.0);
    declare_parameter("field_offset", 3.0);
    declare_parameter("h_low",   18);
    declare_parameter("h_high",  38);
    declare_parameter("s_low",   80);
    declare_parameter("v_low",   80);
    declare_parameter("min_area", 300.0);

    // ── Parameter ArUco ───────────────────────────────────────────────────
    declare_parameter("aruco_target_id", 7);

    // ── Parameter cmd_vel ─────────────────────────────────────────────────
    declare_parameter("linear_speed",   0.5);
    declare_parameter("angular_speed",  2.0);
    declare_parameter("goal_tolerance", 0.15);
    declare_parameter("angular_kp",     2.0);
    declare_parameter("linear_kp",      0.6);

    // ── Parameter Arc Motion ──────────────────────────────────────────────
    declare_parameter("align_enter_tol", 0.12);
    declare_parameter("align_exit_tol",  0.30);
    declare_parameter("arc_blend", 0.7);
    declare_parameter("vel_alpha", 0.35);

    // ── Parameter obstacle (inverse mask) ─────────────────────────────────
    declare_parameter("obstacle_min_area",    500.0);
    declare_parameter("obstacle_slow_factor", 0.3);
    declare_parameter("obstacle_dodge_speed", 0.8);

    // ── Baca semua parameter ──────────────────────────────────────────────
    field_size_         = get_parameter("field_size").as_double();
    field_offset_       = get_parameter("field_offset").as_double();
    h_low_              = get_parameter("h_low").as_int();
    h_high_             = get_parameter("h_high").as_int();
    s_low_              = get_parameter("s_low").as_int();
    v_low_              = get_parameter("v_low").as_int();
    min_area_           = get_parameter("min_area").as_double();
    aruco_target_id_    = get_parameter("aruco_target_id").as_int();
    linear_speed_       = get_parameter("linear_speed").as_double();
    angular_speed_      = get_parameter("angular_speed").as_double();
    goal_tolerance_     = get_parameter("goal_tolerance").as_double();
    angular_kp_         = get_parameter("angular_kp").as_double();
    linear_kp_          = get_parameter("linear_kp").as_double();
    align_enter_tol_    = get_parameter("align_enter_tol").as_double();
    align_exit_tol_     = get_parameter("align_exit_tol").as_double();
    arc_blend_          = get_parameter("arc_blend").as_double();
    vel_alpha_          = get_parameter("vel_alpha").as_double();
    obstacle_min_area_    = get_parameter("obstacle_min_area").as_double();
    obstacle_slow_factor_ = get_parameter("obstacle_slow_factor").as_double();
    obstacle_dodge_speed_ = get_parameter("obstacle_dodge_speed").as_double();

    // ── Setup ArUco ───────────────────────────────────────────────────────
    aruco_dict_   = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
    aruco_params_ = cv::aruco::DetectorParameters::create();
    aruco_params_->adaptiveThreshWinSizeMin  = 3;
    aruco_params_->adaptiveThreshWinSizeMax  = 53;
    aruco_params_->adaptiveThreshWinSizeStep = 4;
    aruco_params_->minMarkerPerimeterRate    = 0.02;
    aruco_params_->maxMarkerPerimeterRate    = 4.0;
    aruco_params_->errorCorrectionRate       = 1.0;

    // ── Subscribers ───────────────────────────────────────────────────────
    sub_image_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&TargetDetector::imageCb, this, std::placeholders::_1));

    // ── Publishers ────────────────────────────────────────────────────────
    pub_target_  = create_publisher<geometry_msgs::msg::Point>("/target_position", 10);
    pub_aruco_   = create_publisher<geometry_msgs::msg::Point>("/aruco_position",  10);
    pub_cmdvel_  = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",         10);
    pub_debug_   = create_publisher<sensor_msgs::msg::Image>("/debug_target",      10);

    RCLCPP_INFO(get_logger(),
        "[REAL LIFE] target_detector started\n"
        "  HSV Kuning H:[%d-%d] S>%d V>%d | min_area=%.0f | ArUco ID=%d\n"
        "  speed: lin=%.1f ang=%.1f | kp: lin=%.1f ang=%.1f\n"
        "  arc_blend=%.1f | vel_alpha=%.2f\n"
        "  align_enter=%.2f align_exit=%.2f\n"
        "  [OBSTACLE] inverse mask | min_area=%.0f | slow=%.1f | dodge=%.1f",
        h_low_, h_high_, s_low_, v_low_, min_area_, aruco_target_id_,
        linear_speed_, angular_speed_, linear_kp_, angular_kp_,
        arc_blend_, vel_alpha_,
        align_enter_tol_, align_exit_tol_,
        obstacle_min_area_, obstacle_slow_factor_, obstacle_dodge_speed_);
  }

  ~TargetDetector()
  {
    geometry_msgs::msg::Twist stop;
    pub_cmdvel_->publish(stop);
    cv::destroyAllWindows();
  }

private:
  // ── Utility ───────────────────────────────────────────────────────────────
  static double clamp(double val, double limit)
  {
    return std::max(-limit, std::min(limit, val));
  }

  static double lowpass(double prev, double target, double alpha)
  {
    return prev + alpha * (target - prev);
  }

  // ── Struct obstacle ───────────────────────────────────────────────────────
  struct ObstacleResult {
    bool        detected;
    double      area;
    cv::Point   center;
    std::string status;   // "CLEAR" | "LEFT" | "RIGHT" | "FRONT"
  };

  // ── Deteksi obstacle: inverse mask (bukan kuning/hijau/putih) ─────────────
 ObstacleResult detectObstacle(const cv::Mat& hsv, cv::Mat& frame, int W, int H)
{
  ObstacleResult obs;
  obs.detected = false;
  obs.area     = 0;
  obs.center   = cv::Point(0, 0);
  obs.status   = "CLEAR";

  // ── Deteksi MERAH secara eksplisit (dual-range karena wrap-around HSV) ──
  cv::Mat mask_red1, mask_red2, mask_red;
  cv::inRange(hsv, cv::Scalar(0,   100, 50), cv::Scalar(10,  255, 255), mask_red1);
  cv::inRange(hsv, cv::Scalar(160, 100, 50), cv::Scalar(179, 255, 255), mask_red2);
  mask_red = mask_red1 | mask_red2;

  cv::Mat mask_obstacle = mask_red.clone();

  // ── Morphological cleanup ──
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
  cv::morphologyEx(mask_obstacle, mask_obstacle, cv::MORPH_OPEN,  kernel);
  cv::morphologyEx(mask_obstacle, mask_obstacle, cv::MORPH_CLOSE, kernel);

  // ── Cari contour terbesar ──
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask_obstacle, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (!contours.empty()) {
    auto largest = std::max_element(contours.begin(), contours.end(),
        [](const auto& a, const auto& b) {
          return cv::contourArea(a) < cv::contourArea(b);
        });

    double area = cv::contourArea(*largest);
    if (area > obstacle_min_area_) {
      obs.detected = true;
      obs.area     = area;

      cv::Moments M = cv::moments(*largest);
      obs.center = cv::Point(
          static_cast<int>(M.m10 / M.m00),
          static_cast<int>(M.m01 / M.m00));

      double rel_x = (obs.center.x - W / 2.0) / (W / 2.0);
      if      (rel_x < -0.3) obs.status = "LEFT";
      else if (rel_x >  0.3) obs.status = "RIGHT";
      else                   obs.status = "FRONT";

      cv::drawContours(frame,
          std::vector<std::vector<cv::Point>>{*largest},
          -1, cv::Scalar(0, 0, 255), 2);
      cv::circle(frame, obs.center, 8, cv::Scalar(0, 0, 255), -1);
      cv::putText(frame,
          "OBSTACLE " + obs.status + " area=" + std::to_string((int)area),
          cv::Point(obs.center.x + 10, obs.center.y - 10),
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
    }
  }

  // Mini preview mask di pojok kanan bawah
  cv::Mat obs_bgr;
  cv::cvtColor(mask_obstacle, obs_bgr, cv::COLOR_GRAY2BGR);
  cv::resize(obs_bgr, obs_bgr, cv::Size(160, 120));
  cv::rectangle(obs_bgr, cv::Point(0, 0), cv::Point(159, 119), cv::Scalar(0, 0, 255), 2);
  cv::putText(obs_bgr, "RED MASK", cv::Point(5, 15),
              cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
  cv::Rect mini_roi(W - 170, H - 130, 160, 120);
  obs_bgr.copyTo(frame(mini_roi));

  return obs;
}

  // ── Hitung faktor kecepatan berdasarkan obstacle ──────────────────────────
  double computeObstacleFactor(const ObstacleResult& obs)
  {
    if (!obs.detected) return 1.0;

    double area_factor = std::max(0.0, 1.0 - (obs.area / 5000.0));
    double final_factor = obstacle_slow_factor_ +
                          (1.0 - obstacle_slow_factor_) * area_factor;
    return std::max(obstacle_slow_factor_, std::min(1.0, final_factor));
  }

  // ── Hitung angular dodge dari obstacle ────────────────────────────────────
  double computeDodgeAngular(const ObstacleResult& obs)
  {
    if (!obs.detected) return 0.0;

    if      (obs.status == "LEFT")  return -obstacle_dodge_speed_; // belok kanan
    else if (obs.status == "RIGHT") return  obstacle_dodge_speed_; // belok kiri
    else                            return -obstacle_dodge_speed_; // FRONT → kanan default
  }

  // ── Callback utama ────────────────────────────────────────────────────────
  void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
      return;
    }

    cv::Mat frame = cv_ptr->image;
    const int W = frame.cols;
    const int H = frame.rows;

    // =========================================================
    // BAGIAN 1 — Deteksi Bola Kuning
    // =========================================================
    cv::Mat blurred, hsv, mask;
    cv::GaussianBlur(frame, blurred, cv::Size(5, 5), 0);
    cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

    cv::inRange(hsv,
        cv::Scalar(h_low_,  s_low_, v_low_),
        cv::Scalar(h_high_, 255,    255),
        mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    bool      ball_detected = false;
    cv::Point ball_pixel;
    double    ball_x_m = 0.0, ball_y_m = 0.0;

    if (!contours.empty()) {
      auto largest = std::max_element(contours.begin(), contours.end(),
          [](const auto& a, const auto& b){
            return cv::contourArea(a) < cv::contourArea(b);
          });

      double area = cv::contourArea(*largest);
      if (area > min_area_) {
        ball_detected = true;

        cv::Moments M = cv::moments(*largest);
        int cx = static_cast<int>(M.m10 / M.m00);
        int cy = static_cast<int>(M.m01 / M.m00);
        ball_pixel = cv::Point(cx, cy);

        ball_x_m = (static_cast<double>(cx) / W) * field_size_ - field_offset_;
        ball_y_m = field_offset_ - (static_cast<double>(cy) / H) * field_size_;

        geometry_msgs::msg::Point pt;
        pt.x = ball_x_m; pt.y = ball_y_m; pt.z = 0.0;
        pub_target_->publish(pt);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "Bola kuning | pixel(%d,%d) -> world(%.2f, %.2f) | area=%.0f",
            cx, cy, ball_x_m, ball_y_m, area);

        cv::Point2f center; float radius;
        cv::minEnclosingCircle(*largest, center, radius);
        cv::circle(frame, center, static_cast<int>(radius), cv::Scalar(0, 255, 255), 3);
        cv::circle(frame, ball_pixel, 7, cv::Scalar(0, 200, 0), -1);
        cv::drawContours(frame,
            std::vector<std::vector<cv::Point>>{*largest},
            -1, cv::Scalar(0, 255, 255), 2);

        std::string lbl = "BOLA (" +
            std::to_string(ball_x_m).substr(0, 5) + ", " +
            std::to_string(ball_y_m).substr(0, 5) + ")m";
        cv::putText(frame, lbl, cv::Point(cx + 10, cy - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        cv::putText(frame, "area=" + std::to_string(static_cast<int>(area)),
            cv::Point(cx + 10, cy + 15),
            cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 0), 1);
      }
    }

    cv::putText(frame,
        ball_detected ? "Bola kuning: TERDETEKSI" : "Bola kuning: TIDAK TERDETEKSI",
        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
        ball_detected ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), 2);

    if (!ball_detected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Bola kuning tidak terdeteksi.");
    }

    // =========================================================
    // BAGIAN 2 — Deteksi ArUco Marker ID 7
    // =========================================================
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners, rejected;

    cv::aruco::detectMarkers(frame, aruco_dict_, marker_corners, marker_ids,
                             aruco_params_, rejected);

    bool      aruco_detected = false;
    cv::Point aruco_pixel;
    double    aruco_x_m = 0.0, aruco_y_m = 0.0;

    if (!marker_ids.empty()) {
      cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);

      std::string all_ids = "Marker: ";
      for (int id : marker_ids) all_ids += std::to_string(id) + " ";
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "%s", all_ids.c_str());

      for (size_t i = 0; i < marker_ids.size(); ++i) {
        if (marker_ids[i] != aruco_target_id_) continue;

        aruco_detected = true;
        const auto& corners = marker_corners[i];
        float cx = (corners[0].x + corners[1].x + corners[2].x + corners[3].x) / 4.0f;
        float cy = (corners[0].y + corners[1].y + corners[2].y + corners[3].y) / 4.0f;
        aruco_pixel = cv::Point(static_cast<int>(cx), static_cast<int>(cy));

        aruco_x_m = (static_cast<double>(cx) / W) * field_size_ - field_offset_;
        aruco_y_m = field_offset_ - (static_cast<double>(cy) / H) * field_size_;

        geometry_msgs::msg::Point pt;
        pt.x = aruco_x_m; pt.y = aruco_y_m; pt.z = 0.0;
        pub_aruco_->publish(pt);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "ArUco ID=%d | pixel(%d,%d) -> world(%.2f, %.2f)",
            aruco_target_id_, aruco_pixel.x, aruco_pixel.y, aruco_x_m, aruco_y_m);

        cv::circle(frame, aruco_pixel, 8, cv::Scalar(255, 0, 255), -1);
        std::string lbl = "ARUCO#" + std::to_string(aruco_target_id_) +
            " (" + std::to_string(aruco_x_m).substr(0, 5) +
            ", " + std::to_string(aruco_y_m).substr(0, 5) + ")m";
        cv::putText(frame, lbl, cv::Point(aruco_pixel.x + 12, aruco_pixel.y - 12),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
        break;
      }
    }

    cv::putText(frame,
        aruco_detected ? "ArUco ID7: TERDETEKSI" : "ArUco ID7: TIDAK TERDETEKSI",
        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
        aruco_detected ? cv::Scalar(255, 0, 255) : cv::Scalar(0, 0, 255), 2);

    if (!aruco_detected) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "ArUco ID=%d tidak terdeteksi.", aruco_target_id_);
    }

    // =========================================================
    // BAGIAN 2.5 — Deteksi Obstacle (Inverse Mask)
    // =========================================================
    ObstacleResult obs     = detectObstacle(hsv, frame, W, H);
    double obstacle_factor = computeObstacleFactor(obs);
    double dodge_angular   = computeDodgeAngular(obs);

    if (obs.detected) {
      cv::putText(frame,
          "OBSTACLE: " + obs.status +
          " | factor=" + std::to_string(obstacle_factor).substr(0, 4),
          cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
          cv::Scalar(0, 0, 255), 2);
    }

    // =========================================================
    // BAGIAN 3 — Control Law
    // =========================================================
    geometry_msgs::msg::Twist cmd;

    if (aruco_detected && ball_detected)
    {
      cv::line(frame, aruco_pixel, ball_pixel, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
      cv::arrowedLine(frame, aruco_pixel, ball_pixel,
                      cv::Scalar(0, 165, 255), 3, cv::LINE_AA, 0, 0.15);

      cv::Point mid((aruco_pixel.x + ball_pixel.x) / 2,
                    (aruco_pixel.y + ball_pixel.y) / 2);

      double dx       = ball_x_m - aruco_x_m;
      double dy       = ball_y_m - aruco_y_m;
      double distance = std::sqrt(dx * dx + dy * dy);
      double angle    = std::atan2(dy, dx);

      cv::putText(frame,
          "d=" + std::to_string(distance).substr(0, 4) + "m",
          cv::Point(mid.x + 5, mid.y - 5),
          cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 165, 255), 2);
      cv::putText(frame,
          "a=" + std::to_string(angle * 180.0 / M_PI).substr(0, 5) + "deg",
          cv::Point(mid.x + 5, mid.y + 18),
          cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 165, 0), 1);

      if (distance > goal_tolerance_)
      {
        double pixel_dx = static_cast<double>(ball_pixel.x - aruco_pixel.x) / (W * 0.5);
        double abs_err  = std::abs(pixel_dx);

        // Arc Motion hysteresis
        if (is_aligned_) {
          if (abs_err > align_exit_tol_)  is_aligned_ = false;
        } else {
          if (abs_err < align_enter_tol_) is_aligned_ = true;
        }

        // Angular: tracking + dodge obstacle
        double raw_angular   = clamp(-angular_kp_ * pixel_dx, angular_speed_);
        double total_angular = clamp(raw_angular + dodge_angular, angular_speed_);

        // Linear: arc blend + obstacle factor
        double base_linear = clamp(linear_kp_ * distance, linear_speed_);
        double aligned_factor;
        if (is_aligned_) {
          aligned_factor = 1.0;
        } else {
          double straightness = std::max(0.0, 1.0 - abs_err / align_exit_tol_);
          aligned_factor = arc_blend_ * straightness;
        }

        double raw_linear = base_linear * aligned_factor * obstacle_factor;

        // Emergency stop kalau obstacle tepat di depan dan sangat dekat
        if (obs.detected && obs.status == "FRONT" && obstacle_factor < 0.5) {
          raw_linear = 0.0;
        }

        // Velocity smoothing
        smooth_linear_  = lowpass(smooth_linear_,  raw_linear,    vel_alpha_);
        smooth_angular_ = lowpass(smooth_angular_, total_angular, vel_alpha_);

        cmd.linear.x  = smooth_linear_;
        cmd.angular.z = smooth_angular_;

        std::string mode_str = std::string(obs.detected ? "AVOID+" : "") +
            std::string(is_aligned_ ? "APPROACH" : "ARC");

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 300,
            "cmd_vel | %s | lin=%.2f ang=%.2f | dist=%.2fm | err_px=%.3f | obs_factor=%.2f",
            mode_str.c_str(),
            cmd.linear.x, cmd.angular.z,
            distance, abs_err, obstacle_factor);

        cv::Scalar mode_color = obs.detected ? cv::Scalar(0, 0, 255) :
                                (is_aligned_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255));
        cv::putText(frame,
            mode_str +
            " | lin=" + std::to_string(cmd.linear.x).substr(0, 4) +
            " ang=" + std::to_string(cmd.angular.z).substr(0, 5),
            cv::Point(10, H - 15),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, mode_color, 2);

      } else {
        // Goal reached
        smooth_linear_  = 0.0;
        smooth_angular_ = 0.0;
        is_aligned_     = false;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "GOAL REACHED (dist=%.2fm < tol=%.2fm). STOP.", distance, goal_tolerance_);

        cv::putText(frame, "GOAL REACHED — STOP",
            cv::Point(10, H - 15),
            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
      }
    }
    else
    {
      smooth_linear_  = lowpass(smooth_linear_,  0.0, vel_alpha_ * 2.0);
      smooth_angular_ = lowpass(smooth_angular_, 0.0, vel_alpha_ * 2.0);
      cmd.linear.x  = smooth_linear_;
      cmd.angular.z = smooth_angular_;
      is_aligned_ = false;

      std::string stop_reason;
      if (!aruco_detected && !ball_detected) stop_reason = "ArUco & Bola tidak terdeteksi";
      else if (!aruco_detected)              stop_reason = "ArUco tidak terdeteksi";
      else                                   stop_reason = "Bola tidak terdeteksi";

      cv::putText(frame, "STOP: " + stop_reason,
          cv::Point(10, H - 15), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 255), 2);
    }

    pub_cmdvel_->publish(cmd);

    // =========================================================
    // BAGIAN 4 — Display OpenCV window
    // =========================================================
    cv::Mat mask_bgr, frame_resized, mask_resized, combined;
    cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
    cv::resize(frame,    frame_resized, cv::Size(640, 480));
    cv::resize(mask_bgr, mask_resized,  cv::Size(640, 480));
    cv::hconcat(mask_resized, frame_resized, combined);

    cv::putText(combined, "HSV MASK (Kuning)",
        cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
    cv::putText(combined, "REAL LIFE: Arc + Inverse Obstacle",
        cv::Point(650, 25), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

    cv::imshow("Target Detector [REAL LIFE]", combined);
    cv::waitKey(1);

    // =========================================================
    // BAGIAN 5 — Publish debug image
    // =========================================================
    auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
    pub_debug_->publish(*debug_msg);
  }

  // ── Subscribers & Publishers ──────────────────────────────────────────────
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr  pub_target_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr  pub_aruco_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr  pub_cmdvel_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr    pub_debug_;

  // ── Parameter bola kuning ─────────────────────────────────────────────────
  double field_size_, field_offset_;
  int    h_low_, h_high_, s_low_, v_low_;
  double min_area_;

  // ── Parameter ArUco ───────────────────────────────────────────────────────
  int aruco_target_id_;
  cv::Ptr<cv::aruco::Dictionary>         aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> aruco_params_;

  // ── Parameter cmd_vel ─────────────────────────────────────────────────────
  double linear_speed_, angular_speed_, goal_tolerance_;
  double angular_kp_, linear_kp_;
  double align_enter_tol_, align_exit_tol_;
  double arc_blend_;
  double vel_alpha_;

  // ── Parameter obstacle ────────────────────────────────────────────────────
  double obstacle_min_area_;
  double obstacle_slow_factor_;
  double obstacle_dodge_speed_;

  // ── State ─────────────────────────────────────────────────────────────────
  bool   is_aligned_     = false;
  double smooth_linear_  = 0.0;
  double smooth_angular_ = 0.0;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetDetector>());
  rclcpp::shutdown();
  return 0;
}