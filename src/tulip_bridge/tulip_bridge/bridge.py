import json
import math
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class TulipBridge(Node):
    def __init__(self):
        super().__init__("tulip_bridge")

        self.declare_parameter("esp_ip", "192.168.4.1")
        self.declare_parameter("esp_port", 8888)
        self.declare_parameter("cmd_timeout", 0.5)

        esp_ip = self.get_parameter("esp_ip").value
        esp_port = self.get_parameter("esp_port").value
        cmd_timeout = self.get_parameter("cmd_timeout").value

        self.esp_addr = (esp_ip, esp_port)
        self.cmd_timeout = cmd_timeout
        self.last_cmd_time = 0.0
        self.last_esp_time = time.time()

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.1)
        self.sock.bind(("0.0.0.0", 0))
        self.get_logger().info(f"UDP bound to port {self.sock.getsockname()[1]}")

        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/tulip/imu", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.have_imu = False

        self.recv_running = True
        self.recv_thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.recv_thread.start()

        self.get_logger().info(f"Bridge started → {esp_ip}:{esp_port}")

    # ── send /cmd_vel to ESP32 ──────────────────────────────────
    def cmd_callback(self, msg: Twist):
        data = json.dumps({
            "linear": round(msg.linear.x, 3),
            "angular": round(msg.angular.z, 3),
        })
        try:
            self.sock.sendto(data.encode(), self.esp_addr)
            self.last_cmd_time = time.time()
        except OSError as e:
            self.get_logger().error(f"UDP send: {e}")

    def timer_callback(self):
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            try:
                self.sock.sendto(
                    json.dumps({"linear": 0.0, "angular": 0.0}).encode(),
                    self.esp_addr,
                )
            except OSError:
                pass

    # ── recv IMU + odometry from ESP32 ──────────────────────────
    def recv_loop(self):
        while self.recv_running and rclpy.ok():
            try:
                data, _ = self.sock.recvfrom(1024)
                now = time.time()
                d = json.loads(data.decode())
            except socket.timeout:
                continue
            except (json.JSONDecodeError, OSError):
                continue

            # ── odometry integration ──
            dt = min(now - self.last_esp_time, 0.5)
            self.last_esp_time = now

            yaw_deg = d.get("yaw", 0.0)
            linear_vel = d.get("linear_vel", 0.0)
            angular_vel = d.get("angular_vel", 0.0)

            # update yaw from IMU (more accurate)
            yaw = math.radians(yaw_deg)
            if self.have_imu:
                v = linear_vel
                self.odom_x += v * math.cos(yaw) * dt
                self.odom_y += v * math.sin(yaw) * dt
            self.odom_yaw = yaw
            self.have_imu = True

            # ── publish /odom ──
            odom = Odometry()
            odom.header = Header()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.twist.twist.linear.x = linear_vel
            odom.twist.twist.angular.z = angular_vel
            qy = math.sin(yaw * 0.5)
            qw = math.cos(yaw * 0.5)
            odom.pose.pose.orientation.z = qy
            odom.pose.pose.orientation.w = qw
            self.odom_pub.publish(odom)

            # ── publish /tulip/imu ──
            roll = math.radians(d.get("roll", 0.0))
            pitch = math.radians(d.get("pitch", 0.0))
            imu_msg = Imu()
            imu_msg.header = Header()
            imu_msg.header.stamp = odom.header.stamp
            imu_msg.header.frame_id = "imu_link"
            cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
            cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
            imu_msg.orientation.w = qw * cp * cr + qy * sp * sr
            imu_msg.orientation.x = qw * cp * sr - qy * sp * cr
            imu_msg.orientation.y = qy * cp * sr + qw * sp * cr
            imu_msg.orientation.z = qy * cp * cr - qw * sp * sr
            self.imu_pub.publish(imu_msg)

    def destroy_node(self):
        self.recv_running = False
        self.sock.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TulipBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
