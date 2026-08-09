#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <chrono>

class UdpBridgeNode : public rclcpp::Node
{
public:
    UdpBridgeNode() : Node("udp_bridge")
    {
        declare_parameter("esp_ip", "172.20.10.4");
        declare_parameter("esp_port", 8888);
        declare_parameter("local_port", 9999);

        esp_ip_ = get_parameter("esp_ip").as_string();
        esp_port_ = get_parameter("esp_port").as_int();
        local_port_ = get_parameter("local_port").as_int();

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&UdpBridgeNode::cmdVelCallback, this, std::placeholders::_1));

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

        setupUdp();

        imu_timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&UdpBridgeNode::receiveImu, this));

        RCLCPP_INFO(get_logger(), "=== UDP Bridge Node Started ===");
        RCLCPP_INFO(get_logger(), "ESP32 IP: %s:%d", esp_ip_.c_str(), esp_port_);
        RCLCPP_INFO(get_logger(), "Listening IMU on port: %d", local_port_);
    }

    ~UdpBridgeNode()
    {
        close(sockfd_);
    }

private:
    void setupUdp()
    {
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to create socket");
            return;
        }

        int broadcast = 1;
        int reuse = 1;
        setsockopt(sockfd_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
        setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct sockaddr_in local_addr;
        memset(&local_addr, 0, sizeof(local_addr));
        local_addr.sin_family = AF_INET;
        local_addr.sin_addr.s_addr = INADDR_ANY;
        local_addr.sin_port = htons(local_port_);

        if (bind(sockfd_, (struct sockaddr*)&local_addr, sizeof(local_addr)) < 0) {
            RCLCPP_ERROR(get_logger(), "Bind failed, errno: %d", errno);
        }

        memset(&esp_addr_, 0, sizeof(esp_addr_));
        esp_addr_.sin_family = AF_INET;
        esp_addr_.sin_port = htons(esp_port_);
        inet_pton(AF_INET, esp_ip_.c_str(), &esp_addr_.sin_addr);
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Delay artificial 20ms (sesuaikan dengan real life)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        char json_buf[128];
        snprintf(json_buf, sizeof(json_buf),
            "{\"linear\":%.3f,\"angular\":%.3f}",
            msg->linear.x, msg->angular.z);

        int sent = sendto(sockfd_, json_buf, strlen(json_buf), 0,
               (struct sockaddr*)&esp_addr_, sizeof(esp_addr_));
        
        if (sent > 0) {
            RCLCPP_DEBUG(get_logger(), "UDP -> ESP32: %s", json_buf);
        }
    }

    void receiveImu()
    {
        char buf[256];
        struct sockaddr_in from_addr;
        socklen_t from_len = sizeof(from_addr);

        int len = recvfrom(sockfd_, buf, sizeof(buf)-1, MSG_DONTWAIT,
                          (struct sockaddr*)&from_addr, &from_len);

        if (len > 0) {
            buf[len] = '\0';
            
            float roll = 0, pitch = 0, yaw = 0;
            int n = sscanf(buf, "{\"roll\":%f,\"pitch\":%f,\"yaw\":%f}", &roll, &pitch, &yaw);
            
            if (n == 3) {
                sensor_msgs::msg::Imu imu_msg;
                imu_msg.header.stamp = now();
                imu_msg.header.frame_id = "imu_link";
                
                double cy = cos(yaw * 0.5);
                double sy = sin(yaw * 0.5);
                double cp = cos(pitch * 0.5);
                double sp = sin(pitch * 0.5);
                double cr = cos(roll * 0.5);
                double sr = sin(roll * 0.5);

                imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
                imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
                imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
                imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

                imu_pub_->publish(imu_msg);
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr imu_timer_;

    int sockfd_;
    struct sockaddr_in esp_addr_;
    std::string esp_ip_;
    int esp_port_, local_port_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UdpBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
