import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial

class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            self.get_logger().info("Conectado a la ESP32")
        except:
            self.ser = None
            self.get_logger().warn("Modo simulación: ESP32 no detectada")

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.arm_sub = self.create_subscription(String, '/arm_cmd', self.arm_callback, 10)

    def cmd_callback(self, msg):
        linear = max(min(msg.linear.x, 1.0), -1.0)
        angular = max(min(msg.angular.z, 1.0), -1.0)
        data = f"W,{linear:.2f},{angular:.2f}\n"
        self._send_serial(data)

    def arm_callback(self, msg):
        data = f"A,{msg.data}\n"
        self._send_serial(data)

    def _send_serial(self, data):
        if self.ser:
            try:
                self.ser.write(data.encode())
            except Exception as e:
                self.get_logger().error(f"Error serial: {e}")

    def destroy_node(self):
        if self.ser:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
