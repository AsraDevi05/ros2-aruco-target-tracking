import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3


class OdomImuPublisher(Node):
    def __init__(self):
        super().__init__('odom_imu_publisher')
        self.last_pose = None

        self.sub = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self._odom_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self._publish_tf)
        self.get_logger().info('Menunggu odometry...')

    def _odom_callback(self, msg):
        now = self.get_clock().now().to_msg()
        self.last_pose = msg.pose.pose

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'chassis'
        odom.pose = msg.pose
        odom.twist = msg.twist
        self.odom_pub.publish(odom)

        self.get_logger().info('Odom pertama diterima - publish tf')

        self._publish_tf()

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'chassis'
        imu.orientation = msg.pose.pose.orientation
        imu.angular_velocity = msg.twist.twist.angular
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = 0.0

        imu.orientation_covariance[0] = 0.01
        imu.orientation_covariance[4] = 0.01
        imu.orientation_covariance[8] = 0.01
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu)

    def _publish_tf(self):
        if self.last_pose is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'chassis'
        t.transform.translation = Vector3(x=self.last_pose.position.x, y=self.last_pose.position.y, z=self.last_pose.position.z)
        t.transform.rotation = self.last_pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
