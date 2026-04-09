import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # Suscripciones
        self.subscription = self.create_subscription(String, '/detections', self.detection_callback, 10)
        self.state_sub = self.create_subscription(String, '/set_state', self.state_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # Publicadores
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm_cmd', 10)

        # Variables de estado y lógica
        self.state = "explore"
        self.last_detection = None
        self.last_time = time.time()
        self.targets = []
        self.map_rocks = [] # Inicialización necesaria para evitar errores
        self.current_target = None
        self.front_distance = 999.0

        self.collected_rocks = 0
        self.max_rocks = 10

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self.last_action_time = time.time()
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.mission_start = time.time()
        self.max_mission_time = 480 
        self.panel_height = None 

        self.get_logger().info("FAT RAT - Mission Node con Evasión de Obstáculos Iniciado")
        self.timer = self.create_timer(0.1, self.loop)

    def scan_callback(self, msg):
        """Procesa datos del LiDAR para detectar obstáculos frontales."""
        center = len(msg.ranges) // 2
        window = 20 # Rango de grados al centro

        valid = [
            r for r in msg.ranges[center-window:center+window]
            if math.isfinite(r)
        ]

        if valid:
            self.front_distance = min(valid)

    def state_callback(self, msg):
        self.state = msg.data
        self.last_action_time = time.time()
        self.get_logger().info(f"*** ESTADO CAMBIADO A: {self.state} ***")

    def get_lidar_distance(self, angle_rad):
        return self.front_distance

    def detection_callback(self, msg):
        data = msg.data.split(',')

        # Ahora esperamos 6 campos
        if len(data) != 6:
            return

        label, cx, color, tamano, textura, forma = data

        if label != "roca":
            return

        cx = int(cx)
        width = 640
        fov = 60.0
        angle = (cx - width/2) * (fov / width)
        angle_rad = math.radians(angle)

        distance = self.get_lidar_distance(angle_rad)

        if distance is None or distance > 3.0:
            return

        rock_x = self.x + distance * math.cos(self.theta + angle_rad)
        rock_y = self.y + distance * math.sin(self.theta + angle_rad)

        rock = {
            "x": round(rock_x, 2),
            "y": round(rock_y, 2),
            "color": color,
            "tamano": tamano,
            "forma": forma,
            "textura": textura,
            "distance": round(distance, 2),
            "timestamp": time.time()
        }

        for r in self.map_rocks:
            if math.hypot(r["x"] - rock["x"], r["y"] - rock["y"]) < 0.3:
                return

        self.map_rocks.append(rock)
        self.get_logger().info(f"Roca registrada: {rock}")

    def update_pose_from_tf(self):
        try:
            trans = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.theta = math.atan2(siny_cosp, cosy_cosp)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def send_arm_to_height(self, target_z):
        z_clamped = max(0, min(target_z, 1000))
        pwm_hombro = int(110 + (z_clamped / 1000.0) * (510 - 110))
        pwm_codo = int(510 - (z_clamped / 1000.0) * (510 - 200))
        self.arm_pub.publish(String(data=f"ARM:SET:1,{pwm_hombro}"))
        self.arm_pub.publish(String(data=f"ARM:SET:2,{pwm_codo}"))

    def loop(self):
        msg = Twist()

        if time.time() - self.mission_start > self.max_mission_time:
            self.state = "return"
        
        if time.time() - self.last_action_time > 12 and self.state in ["explore", "approach"]:
            self.state = "explore"
            self.targets.clear()
            self.current_target = None
            self.last_action_time = time.time()

        # --- MÁQUINA DE ESTADOS ---
        if self.state == "explore":
            # Lógica de evasión con LiDAR
            if self.front_distance < 0.6:
                msg.linear.x = 0.0
                msg.angular.z = 0.4 # Giro para evitar obstáculo
            else:
                msg.linear.x = 0.18
                msg.angular.z = 0.25
            
            if len(self.targets) > 5:
                CENTER_X = 320
                self.current_target = min(self.targets, key=lambda x: abs(x - CENTER_X))
                self.state = "approach"
                self.last_action_time = time.time()

        elif self.state == "approach":
            if time.time() - self.last_time > 1.5:
                self.state = "explore"
                return

            cx = self.current_target
            error = cx - 320
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
            t_col = time.time() - self.last_action_time
            if t_col < 0.5: 
                self.arm_pub.publish(String(data="ARM:DEPLOY"))
            elif 2.0 < t_col < 2.5: 
                self.arm_pub.publish(String(data="ARM:STOW"))
            elif t_col > 4.0:
                self.collected_rocks += 1
                self.targets.clear()
                self.state = "explore" 
                self.last_action_time = time.time()

        elif self.state == "return":
            dx, dy = self.home_x - self.x, self.home_y - self.y
            dist = math.sqrt(dx**2 + dy**2)
            error_a = math.atan2(dy, dx) - self.theta
            error_a = math.atan2(math.sin(error_a), math.cos(error_a))

            if dist > 0.2:
                msg.linear.x = 0.15
                msg.angular.z = max(min(error_a, 0.4), -0.4)
            else:
                self.state = "deposit"
                self.last_action_time = time.time()

        elif self.state == "deposit":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            t_dep = time.time() - self.last_action_time
            if t_dep < 1.0:
                self.arm_pub.publish(String(data="ARM:SET:1,450"))
                self.arm_pub.publish(String(data="ARM:SET:2,200"))
            elif 3.0 < t_dep < 3.5:
                self.arm_pub.publish(String(data="ARM:SET:5,110"))
            elif t_dep > 5.0:
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "finished"

        elif self.state == "mantenimiento":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if self.panel_height is None: 
                return
            t_mant = time.time() - self.last_action_time
            bz = self.panel_height 
            
            if t_mant < 1.0: self.send_arm_to_height(bz + 50) 
            elif 3.0 < t_mant < 3.5: self.send_arm_to_height(bz - 20) 
            elif 6.0 < t_mant < 6.5: self.send_arm_to_height(bz + 80) 
            elif 9.0 < t_mant < 9.5: self.send_arm_to_height(bz - 50) 
            elif 12.0 < t_mant < 12.5:
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "finished"

        elif self.state == "finished":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        if abs(msg.angular.z) > 0.5:
            msg.angular.z = 0.5 * (msg.angular.z / abs(msg.angular.z))

        self.update_pose_from_tf()
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
