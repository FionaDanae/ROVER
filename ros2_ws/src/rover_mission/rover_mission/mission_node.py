import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import math

class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.subscription = self.create_subscription(String, '/detections', self.detection_callback, 10)
        self.state_sub = self.create_subscription(String, '/set_state', self.state_callback, 10)
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm_cmd', 10)

        self.state = "explore"
        self.last_detection = None
        self.last_time = time.time()
        self.detection_count = 0
        self.targets = []
        self.current_target = None
        
        self.collected_rocks = 0
        self.max_rocks = 3

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.last_action_time = time.time()

        self.get_logger().info("Mission Node FINAL iniciado - Autonomía TMR 2026")
        self.timer = self.create_timer(0.1, self.loop)

    def state_callback(self, msg):
        self.state = msg.data
        self.last_action_time = time.time()
        self.get_logger().info(f"*** ESTADO CAMBIADO MANUALMENTE A: {self.state} ***")

    def detection_callback(self, msg):
        if not msg.data:
            return
        
        self.last_detection = msg.data
        self.last_time = time.time()

        try:
            data_parts = msg.data.split(',')
            label = data_parts[0]
            cx = int(data_parts[1]) if len(data_parts) > 1 and data_parts[1] != "none" else 0
            
            if "fin" in msg.data.lower():
                self.get_logger().info("FIN detectado → Iniciando retorno a la base")
                self.state = "return"
                return

            if "roca" in label.lower() or "rojo" in msg.data.lower():
                self.targets.append(cx)
                if len(self.targets) > 10:
                    self.targets.pop(0)

        except Exception as e:
            pass

    def update_pose(self, linear, angular, dt=0.1):
        self.theta += angular * dt
        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt

    def loop(self):
        msg = Twist()

        if time.time() - self.last_action_time > 12:
            self.state = "explore"
            self.targets.clear()
            self.current_target = None
            self.last_action_time = time.time()

        if self.state == "explore":
            msg.linear.x = 0.18
            msg.angular.z = 0.25
            if len(self.targets) > 5:
                self.current_target = min(self.targets, key=lambda x: abs(x - 160))
                self.state = "approach"
                self.last_action_time = time.time()

        elif self.state == "approach":
            if time.time() - self.last_time > 1.5:
                self.state = "explore"
                return

            cx = self.current_target
            error = cx - 160
            linear_speed = max(0.08, 0.25 - abs(error)/200)

            if abs(error) < 15:
                msg.linear.x = linear_speed
                msg.angular.z = 0.0
                if time.time() - self.last_time > 1.8:
                    self.state = "collect"
                    self.last_action_time = time.time()
            else:
                msg.linear.x = 0.0
                msg.angular.z = max(min(-error / 140.0, 0.5), -0.5)

        elif self.state == "collect":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            tiempo_recoleccion = time.time() - self.last_action_time
            
            if tiempo_recoleccion < 0.5:
                self.arm_pub.publish(String(data="ARM:DEPLOY"))
            elif 2.0 < tiempo_recoleccion < 2.5:
                self.arm_pub.publish(String(data="ARM:STOW"))
            elif tiempo_recoleccion > 4.0:
                self.collected_rocks += 1
                self.get_logger().info(f"Roca recolectada ({self.collected_rocks})")
                self.targets.clear()
                self.current_target = None
                self.state = "explore" 
                self.last_action_time = time.time()

        elif self.state == "return":
            dx = self.home_x - self.x
            dy = self.home_y - self.y
            distance = math.sqrt(dx**2 + dy**2)
            angle_error = math.atan2(dy, dx) - self.theta
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

            if distance > 0.2:
                msg.linear.x = 0.15
                msg.angular.z = max(min(angle_error, 0.4), -0.4)
            else:
                self.state = "deposit"
                self.last_action_time = time.time()

        elif self.state == "deposit":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            tiempo_deposito = time.time() - self.last_action_time

            if tiempo_deposito < 1.0:
                self.arm_pub.publish(String(data="ARM:SET:1,450")) 
                self.arm_pub.publish(String(data="ARM:SET:2,200"))
            elif 3.0 < tiempo_deposito < 3.5:
                self.arm_pub.publish(String(data="ARM:SET:5,110")) 
            elif tiempo_deposito > 5.0:
                self.get_logger().info(f"Depósito completado.")
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "finished"

        elif self.state == "mantenimiento":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            t_mant = time.time() - self.last_action_time
            
            if t_mant < 1.0:
                self.get_logger().info("Ejecutando Secuencia en Cápsula...")
                self.arm_pub.publish(String(data="ARM:SET:1,400")) 
            elif 2.0 < t_mant < 2.5:
                self.arm_pub.publish(String(data="ARM:SET:0,350"))
            elif 4.0 < t_mant < 4.5:
                self.arm_pub.publish(String(data="ARM:SET:0,300"))
            elif 6.0 < t_mant < 6.5:
                self.arm_pub.publish(String(data="ARM:SET:0,400"))
            elif 8.0 < t_mant < 8.5:
                self.arm_pub.publish(String(data="ARM:SET:0,450"))
            elif t_mant > 10.0:
                self.get_logger().info("Secuencia finalizada.")
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "finished"

        elif self.state == "finished":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        if abs(msg.angular.z) > 0.5:
            msg.angular.z = 0.5 * (msg.angular.z / abs(msg.angular.z))

        self.update_pose(msg.linear.x, msg.angular.z)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
