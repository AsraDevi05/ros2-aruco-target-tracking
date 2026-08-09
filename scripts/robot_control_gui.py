import tkinter as tk
from tkinter import ttk
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_gui')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            Odometry, '/model/vehicle_blue/odometry', self._odom_callback, 10)
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw = 0.0

    def send_cmd(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)

    def _odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)


class AnalogJoystick(tk.Canvas):
    def __init__(self, parent, size=200, on_move=None, **kwargs):
        super().__init__(parent, width=size, height=size,
                         highlightthickness=0, bg='#1a1a2e', **kwargs)
        self.size = size
        self.on_move = on_move
        self.radius = size // 2 - 10
        self.cx = size // 2
        self.cy = size // 2
        self.thumb_x = self.cx
        self.thumb_y = self.cy
        self.dragging = False

        self._draw_base()
        self.thumb = self.create_oval(
            self.cx - 15, self.cy - 15, self.cx + 15, self.cy + 15,
            fill='#e94560', outline='#ffffff', width=2
        )
        self.create_text(self.cx, self.cy, text='●', fill='#555555',
                         font=('Arial', 8))

        self.bind('<Button-1>', self._on_press)
        self.bind('<B1-Motion>', self._on_drag)
        self.bind('<ButtonRelease-1>', self._on_release)

    def _draw_base(self):
        self.create_oval(
            self.cx - self.radius, self.cy - self.radius,
            self.cx + self.radius, self.cy + self.radius,
            outline='#16213e', fill='#0f3460', width=3
        )
        self.create_line(self.cx - self.radius, self.cy,
                         self.cx + self.radius, self.cy,
                         fill='#16213e', width=1)
        self.create_line(self.cx, self.cy - self.radius,
                         self.cx, self.cy + self.radius,
                         fill='#16213e', width=1)

    def _on_press(self, event):
        self.dragging = True
        self._update_thumb(event.x, event.y)

    def _on_drag(self, event):
        if self.dragging:
            self._update_thumb(event.x, event.y)

    def _on_release(self, event):
        self.dragging = False
        self._update_thumb(self.cx, self.cy)

    def _update_thumb(self, x, y):
        dx = x - self.cx
        dy = y - self.cy
        dist = math.hypot(dx, dy)

        if dist > self.radius:
            dx = dx / dist * self.radius
            dy = dy / dist * self.radius

        self.thumb_x = self.cx + dx
        self.thumb_y = self.cy + dy

        self.coords(self.thumb,
                    self.thumb_x - 15, self.thumb_y - 15,
                    self.thumb_x + 15, self.thumb_y + 15)

        if self.on_move:
            nx = dx / self.radius
            ny = -dy / self.radius
            self.on_move(nx, ny)

    def reset(self):
        self._update_thumb(self.cx, self.cy)


class RobotGUI:
    def __init__(self, node):
        self.node = node
        self.max_speed = 0.5
        self.linear_x = 0.0
        self.angular_z = 0.0

        self.root = tk.Tk()
        self.root.title('Robot Control')
        self.root.resizable(False, False)

        main = ttk.Frame(self.root, padding=15)
        main.pack()

        self.joystick = AnalogJoystick(main, size=200, on_move=self._on_joystick)
        self.joystick.pack(pady=(10, 0))

        self.root.bind('<Escape>', lambda e: self._on_close())
        self.root.protocol('WM_DELETE_WINDOW', self._on_close)

        self._publish_loop()

    def _on_joystick(self, nx, ny):
        self.linear_x = ny * self.max_speed
        self.angular_z = -nx * self.max_speed

    def _publish_loop(self):
        self.node.send_cmd(self.linear_x, self.angular_z)
        self.root.after(50, self._publish_loop)

    def _on_close(self):
        self.node.send_cmd(0.0, 0.0)
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()

    def run(self):
        spin_thread = threading.Thread(target=self._spin, daemon=True)
        spin_thread.start()
        self.root.mainloop()

    def _spin(self):
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)


def main():
    rclpy.init()
    node = RobotControlNode()
    gui = RobotGUI(node)
    gui.run()


if __name__ == '__main__':
    main()
