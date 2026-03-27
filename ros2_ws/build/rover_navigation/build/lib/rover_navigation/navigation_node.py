import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.subscription = self.create_subscription(
            String,
            '/detections',
            self.detection_callback,
            10
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.last_detection = None
        self.last_detection_time = 0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Navigation Node iniciado")

    def detection_callback(self, msg):
        if msg.data is not None and msg.data != "":
            self.last_detection = msg.data
            self.last_detection_time = time.time()

    def control_loop(self):

        msg = Twist()

        if self.last_detection is None or (time.time() - self.last_detection_time > 1.0):

            msg.linear.x = 0.2
            msg.angular.z = 0.3

            self.get_logger().info("Explorando...")

        else:
            self.get_logger().info(f"Detección válida: {self.last_detection}")

            try:
                parts = self.last_detection.split(',')

                if len(parts) != 3:
                    raise ValueError("Formato incorrecto")

                _, cx, color = parts
                cx = int(cx)

                center = 160
                error = cx - center

                # CENTRADO
                if abs(error) < 20:
                    msg.linear.x = 0.25
                    msg.angular.z = 0.0

                    self.get_logger().info("Avanzando hacia roca")

                else:
                    msg.linear.x = 0.0
                    msg.angular.z = -error / 100.0

                    self.get_logger().info(f"Ajustando dirección: {error}")

            except Exception as e:
                self.get_logger().error(f"Error procesando detección: {e}")
                self.last_detection = None  # resetear

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
