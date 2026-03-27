import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class RoverDriver(Node):

    def __init__(self):
        super().__init__('rover_driver')

        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Serial conectado")
        except:
            self.ser = None
            self.get_logger().warn("ESP32 no conectada, modo simulación")

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

    def cmd_callback(self, msg):

        linear = max(min(msg.linear.x, 1.0), -1.0)
        angular = max(min(msg.angular.z, 1.0), -1.0)

        data = f"{linear:.2f},{angular:.2f}\n"

        if self.ser:
            try:
                self.ser.write(data.encode())
            except:
                self.get_logger().error("Error enviando serial")

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RoverDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
