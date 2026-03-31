import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import matplotlib.pyplot as plt
import math

class MonitorNode(Node):
    def __init__(self):
        super().__init__('monitor_node')

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

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.timer = self.create_timer(0.2, self.update_plot)

    def cmd_callback(self, msg):
        dt = 0.2
        self.theta += msg.angular.z * dt
        self.x += msg.linear.x * math.cos(self.theta) * dt
        self.y += msg.linear.x * math.sin(self.theta) * dt
        self.path_x.append(self.x)
        self.path_y.append(self.y)

    def det_callback(self, msg):
        if msg.data and "roca" in msg.data:
            self.objects.append((self.x, self.y, msg.data.split(',')[2])) 

    def terrain_callback(self, msg):
        if msg.data:
            self.terrains.append((self.x, self.y, msg.data))

    def update_plot(self):
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
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = MonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
