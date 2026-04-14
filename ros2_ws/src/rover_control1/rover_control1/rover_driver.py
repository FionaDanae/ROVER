import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time

class HardwareBridge(Node):
    def __init__(self):
        super().__init__('hardware_bridge')
        try:
            self.ser = serial.Serial('/dev/rover_esp32', 115200, timeout=0.05)
            self.get_logger().info("Conectado a la ESP32 (Motores + Servos + IMU)")
        except:
            self.ser = None
            self.get_logger().warn("ESP32 no detectada")

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.arm_sub = self.create_subscription(String, '/arm_cmd', self.arm_callback, 10)
        
        # Publicadores
        self.terrain_pub = self.create_publisher(String, '/terrain_status', 10)
        self.weight_pub = self.create_publisher(String, '/rock_weight', 10)
        
        self.timer = self.create_timer(0.05, self.read_serial)

    def cmd_callback(self, msg):
        linear = max(min(msg.linear.x, 1.0), -1.0)
        angular = max(min(msg.angular.z, 1.0), -1.0)
        data = f"W,{linear:.2f},{angular:.2f}\n"
        self._send_serial(data)

    def arm_callback(self, msg):
        data = f"A,{msg.data}\n"
        self._send_serial(data)

    def read_serial(self):
        if not self.ser: return
        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith("I,"):
                    parts = line.split(',')
                    if len(parts) >= 3:
                        pitch = float(parts[1])
                        roll = float(parts[2])
                        self.analyze_terrain(pitch, roll)
                    if len(parts) >= 4:
                        peso = float(parts[3])
                        msg_peso = String()
                        msg_peso.data = str(peso)
                        self.weight_pub.publish(msg_peso)
        except Exception as e:
            pass

    def analyze_terrain(self, pitch, roll):
        terreno = None
        
        # Ponemos el umbral en 22.0 grados para asegurar la detección cuando el
        # rover esté subiendo o bajando de frente, dando un margen de seguridad.
        if abs(pitch) > 22.0 and abs(roll) < 15.0:
            terreno = "pendiente"
            
        # Un surco suele ser una zanja estrecha. Cuando el rover cae en uno, 
        # normalmente se desestabiliza bruscamente en ambos ejes a la vez.
        elif abs(pitch) >= 15.0 and abs(roll) >= 15.0:
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
        if self.ser:
            try:
                self.ser.write(data.encode())
            except Exception as e:
                pass

    def destroy_node(self):
        if self.ser: self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
