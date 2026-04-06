import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time


class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baudrate', 115200)
        self.declare_parameter('serial_timeout_s', 0.05)
        self.declare_parameter('reconnect_period_s', 2.0)
        self.declare_parameter('read_period_s', 0.05)

        self._serial_port = (
            self.get_parameter('serial_port').get_parameter_value().string_value
        )
        self._serial_baudrate = (
            self.get_parameter('serial_baudrate').get_parameter_value().integer_value
        )
        self._serial_timeout_s = (
            self.get_parameter('serial_timeout_s')
            .get_parameter_value()
            .double_value
        )
        self._reconnect_period_s = (
            self.get_parameter('reconnect_period_s')
            .get_parameter_value()
            .double_value
        )
        self._read_period_s = (
            self.get_parameter('read_period_s').get_parameter_value().double_value
        )

        self.ser = None
        self._last_reconnect_attempt_s = 0.0
        self._connect_serial(log_success=True)

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.arm_sub = self.create_subscription(String, '/arm_cmd', self.arm_callback, 10)

        self.terrain_pub = self.create_publisher(String, '/terrain_status', 10)

        self.timer = self.create_timer(self._read_period_s, self.read_serial)

    def _connect_serial(self, log_success=False):
        if self.ser is not None:
            return True

        now_s = time.monotonic()
        if now_s - self._last_reconnect_attempt_s < self._reconnect_period_s:
            return False

        self._last_reconnect_attempt_s = now_s
        try:
            self.ser = serial.Serial(
                self._serial_port,
                int(self._serial_baudrate),
                timeout=float(self._serial_timeout_s),
            )
        except (serial.SerialException, OSError) as exc:
            self.ser = None
            self.get_logger().warning(f'ESP32 no disponible: {exc}')
            return False

        if log_success:
            self.get_logger().info('Conectado a la ESP32')
        return True

    def cmd_callback(self, msg):
        linear = max(min(msg.linear.x, 1.0), -1.0)
        angular = max(min(msg.angular.z, 1.0), -1.0)
        data = f"W,{linear:.2f},{angular:.2f}\n"
        self._send_serial(data)

    def arm_callback(self, msg):
        data = f"A,{msg.data}\n"
        self._send_serial(data)

    def read_serial(self):
        if not self._connect_serial():
            return
        try:
            while self.ser is not None and self.ser.in_waiting > 0:
                line = (
                    self.ser.readline()
                    .decode('utf-8', errors='replace')
                    .strip()
                )
                if line.startswith("I,"):
                    parts = line.split(',')
                    if len(parts) == 3:
                        pitch = float(parts[1])
                        roll = float(parts[2])
                        self.analyze_terrain(pitch, roll)
        except Exception as exc:
            self.get_logger().warning(f'Error leyendo serial: {exc}')

    def analyze_terrain(self, pitch, roll):
        terreno = None

        if abs(pitch) > 18.0:
            terreno = "pendiente"
        elif abs(roll) > 12.0:
            terreno = "valle"
        elif abs(pitch) > 10.0 and abs(roll) > 10.0:
            terreno = "surco"
            
        # Un valle es una depresión más amplia. Si el rover lo transita de lado,
        # generará una inclinación lateral (roll) pronunciada.
        elif abs(roll) > 18.0 and abs(pitch) < 15.0:
            terreno = "valle"

        if terreno:
            msg = String()
            msg.data = terreno
            self.terrain_pub.publish(msg)
            self.get_logger().info(f"Accidente geográfico detectado por IMU: {terreno} (P:{pitch:.1f}, R:{roll:.1f})")

    def _send_serial(self, data):
        if not self._connect_serial():
            return
        try:
            if self.ser is not None:
                self.ser.write(data.encode())
        except Exception as exc:
            self.get_logger().warning(f'Error escribiendo serial: {exc}')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
