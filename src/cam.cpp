#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/opencv.hpp>

class RobotLocalizer : public rclcpp::Node
{
public:
    RobotLocalizer() : Node("robot_localizer")
    {
        sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&RobotLocalizer::imageCb, this, std::placeholders::_1));

        pub_pos_ = create_publisher<geometry_msgs::msg::Point>("/robot_position", 10);
        pub_debug_ = create_publisher<sensor_msgs::msg::Image>("/debug_image", 10);

        RCLCPP_INFO(get_logger(), "Robot Localizer started");
    }

private:
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge error: %s", e.what());
            return;
        }

        cv::Mat frame = cv_ptr->image;
        cv::Mat hsv, mask;

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        // Masker warna biru robot
        cv::Scalar lower_blue(100, 80, 50);
        cv::Scalar upper_blue(130, 255, 255);
        cv::inRange(hsv, lower_blue, upper_blue, mask);

        // Morphology untuk hapus noise
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_DILATE, kernel);

        // Cari contour
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            // Ambil contour terbesar
            auto largest = std::max_element(contours.begin(), contours.end(),
                [](const auto &a, const auto &b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            if (cv::contourArea(*largest) > 50.0) {
                cv::Moments M = cv::moments(*largest);
                int cx = static_cast<int>(M.m10 / M.m00);
                int cy = static_cast<int>(M.m01 / M.m00);

                int h = frame.rows;
                int w = frame.cols;

                // Konversi pixel → meter (world frame)
                double x_m = (static_cast<double>(cx) / w) * field_size_ - field_offset_;
                double y_m = (static_cast<double>(cy) / h) * field_size_ - field_offset_;

                geometry_msgs::msg::Point pos;
                pos.x = x_m;
                pos.y = y_m;
                pos.z = 0.0;
                pub_pos_->publish(pos);

                RCLCPP_INFO(get_logger(), "Robot pixel (%d,%d) -> world (%.2f, %.2f)", cx, cy, x_m, y_m);

                // Debug image
                cv::drawContours(frame, std::vector<std::vector<cv::Point>>{*largest}, -1, cv::Scalar(0, 255, 0), 2);
                cv::circle(frame, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);
                cv::putText(frame,
                    "(" + std::to_string(x_m).substr(0,5) + ", " + std::to_string(y_m).substr(0,5) + ")",
                    cv::Point(cx + 10, cy),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
            }
        }

        auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        pub_debug_->publish(*debug_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_pos_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_;

    const double field_size_ = 6.0;   // lapangan 6x6 meter
    const double field_offset_ = 3.0; // dari -3 sampai +3
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotLocalizer>());
    rclcpp::shutdown();
    return 0;
}