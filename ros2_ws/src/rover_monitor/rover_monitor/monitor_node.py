import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

        self.declare_parameter('enable_plot', True)
        self.declare_parameter('plot_period_s', 0.2)
        self._enable_plot = (
            self.get_parameter('enable_plot').get_parameter_value().bool_value
        )
        self._plot_period_s = (
            self.get_parameter('plot_period_s').get_parameter_value().double_value
        )

        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.sub_det = self.create_subscription(String, '/detections', self.det_callback, 10)
        self.sub_terrain = self.create_subscription(String, '/terrain_status', self.terrain_callback, 10)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.path_x = []
        self.path_y = []

        self.objects = []
        self.terrains = []

        self._last_cmd_time = self.get_clock().now()
        self._plt = None
        self.fig = None
        self.ax = None

        if self._enable_plot:
            try:
                import matplotlib.pyplot as plt
            except Exception as exc:
                self.get_logger().warning(f'Plot deshabilitado: {exc}')
                self._enable_plot = False
            else:
                self._plt = plt
                self._plt.ion()
                self.fig, self.ax = self._plt.subplots()
                self.timer = self.create_timer(self._plot_period_s, self.update_plot)

    def cmd_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self._last_cmd_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.0
        self._last_cmd_time = now

        self.theta += msg.angular.z * dt
        self.x += msg.linear.x * math.cos(self.theta) * dt
        self.y += msg.linear.x * math.sin(self.theta) * dt
        self.path_x.append(self.x)
        self.path_y.append(self.y)

    def det_callback(self, msg):
        if not msg.data:
            return
        parts = [p.strip() for p in msg.data.split(',')]
        if len(parts) >= 3 and parts[0].lower() == 'roca':
            self.objects.append((self.x, self.y, parts[2]))

    def terrain_callback(self, msg):
        if msg.data:
            self.terrains.append((self.x, self.y, msg.data))

    def update_plot(self):
        if not self._enable_plot or self.ax is None or self._plt is None:
            return

        self.ax.clear()
        self.ax.plot(self.path_x, self.path_y, 'b-', label="Ruta")

        for obj in self.objects:
            x, y, label = obj
            self.ax.scatter(x, y, marker='o', color='green')
            self.ax.text(x, y+0.1, f"Roca {label}", fontsize=8)

        for terr in self.terrains:
            x, y, label = terr
            self.ax.scatter(x, y, marker='^', color='orange')
            self.ax.text(x, y-0.2, label, color='orange', fontsize=8, fontweight='bold')

        self.ax.set_title("Mapa Lunar - FAT RAT")
        self.ax.legend()
        self._plt.draw()
        self._plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
