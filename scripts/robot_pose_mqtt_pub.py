import json
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt


MQTT_TOPIC_POSE = 'robot/pose'


class PoseMqttBridge(Node):
    def __init__(self):
        super().__init__('pose_mqtt_bridge')
        self.declare_parameter('broker', 'localhost')
        self.declare_parameter('port', 1883)

        broker = self.get_parameter('broker').value
        port = self.get_parameter('port').value

        self.mqtt_client = mqtt.Client()
        try:
            self.mqtt_client.connect(broker, port, 60)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'Connected to MQTT broker at {broker}:{port}')
        except Exception as e:
            self.get_logger().error(f'MQTT connection failed: {e}')

        self.sub = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self._odom_callback, 10)

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        payload = json.dumps({'x': x, 'y': y, 'yaw': yaw})
        self.mqtt_client.publish(MQTT_TOPIC_POSE, payload)


def main(args=None):
    rclpy.init(args=args)
    node = PoseMqttBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
