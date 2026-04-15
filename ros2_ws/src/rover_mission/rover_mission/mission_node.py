import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
import math


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('loop_period_s', 0.1)
        self.declare_parameter('image_width_px', 640)
        self.declare_parameter('max_targets', 10)
        self.declare_parameter('target_lock_count', 6)
        self.declare_parameter('action_timeout_s', 12.0)
        self.declare_parameter('max_rocks', 10)

        self._image_width_px = (
            self.get_parameter('image_width_px').get_parameter_value().integer_value
        )
        self._max_targets = (
            self.get_parameter('max_targets').get_parameter_value().integer_value
        )
        self._target_lock_count = (
            self.get_parameter('target_lock_count').get_parameter_value().integer_value
        )
        self._action_timeout_s = (
            self.get_parameter('action_timeout_s').get_parameter_value().double_value
        )
        self.max_rocks = (
            self.get_parameter('max_rocks').get_parameter_value().integer_value
        )

        self.subscription = self.create_subscription(
            String, '/detections', self.detection_callback, 10
        )
        self.state_sub = self.create_subscription(String, '/set_state', self.state_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm_cmd', 10)

        # Variables de estado y lógica
        self.state = "explore"
        self.last_detection = None
        self._last_detection_time_s = 0.0
        self.targets = []
        self.controles_detectados = []
        self.map_rocks = [] # Inicialización necesaria para evitar errores
        self.current_target = None
        self.front_distance = 999.0

        # ATENCIÓN: Necesitas declarar self.last_scan para que get_lidar_distance no falle
        self.last_scan = None 

        self.collected_rocks = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        self._last_action_time_s = time.monotonic()
        self._last_loop_time = self.get_clock().now()

        self.panel_height = None
        self.inicio_cx = None
        self.last_inicio_time = 0.0
        self._last_panel_search_log_s = 0.0

        self.get_logger().info("FAT RAT - Mission Node Iniciado (Modo 100% Autónomo)")
        loop_period_s = (
            self.get_parameter('loop_period_s').get_parameter_value().double_value
        )
        self.timer = self.create_timer(loop_period_s, self.loop)

    def state_callback(self, msg):
        self.state = msg.data
        self._last_action_time_s = time.monotonic()
        self.get_logger().info(f"*** ESTADO CAMBIADO MANUALMENTE A: {self.state} ***")

    def scan_callback(self, msg):
        self.last_scan = msg

    def get_lidar_distance(self, angle_rad):
        if self.last_scan is None:
            return None

        scan = self.last_scan
        index = int((angle_rad - scan.angle_min) / scan.angle_increment)
        if 0 <= index < len(scan.ranges):
            dist = scan.ranges[index]
            if math.isfinite(dist):
                return dist
        return None

    def detection_callback(self, msg):
        data_parts = [p.strip() for p in msg.data.split(',') if p.strip()]
        if not data_parts:
            return

        label = data_parts[0].lower()
        now_s = time.monotonic()

        if label == "persona":
            self.state = "emergency_stop"
            self._last_action_time_s = now_s
            return

        if label == "fin":
            if self.state not in ["return", "celebrate_fin"]:
                self.state = "celebrate_fin"
                self._last_action_time_s = now_s
                self.get_logger().info("¡Letrero FIN detectado! Indicando visualmente.")
            return

        if label == "inicio" and len(data_parts) >= 2:
            try:
                self.inicio_cx = int(data_parts[1])
                self.last_inicio_time = now_s
            except ValueError:
                pass
            return

        if label == "panel" and len(data_parts) >= 3:
            try:
                self.panel_height = int(data_parts[2])
            except ValueError:
                self.panel_height = None
            if self.state in ["explore", "approach"]:
                self.state = "mantenimiento"
                self._last_action_time_s = now_s
                self.targets.clear()
                self.get_logger().info("¡Panel detectado! Iniciando secuencia de mantenimiento.")
            return

        if label == "control" and len(data_parts) >= 4:
            try:
                cx_ctrl = int(data_parts[2])
            except ValueError:
                cx_ctrl = 0
            self.controles_detectados.append({"tipo": data_parts[1], "cx": cx_ctrl, "estado": data_parts[3]})
            return

        if label != "roca" or len(data_parts) < 2:
            return

        try:
            cx = int(data_parts[1])
        except ValueError:
            return

        width = float(self._image_width_px)
        fov = 60.0
        angle = (cx - width / 2.0) * (fov / width)
        angle_rad = math.radians(angle)
        distance = self.get_lidar_distance(angle_rad)
        if distance is not None and distance > 3.0:
            return

        self.last_detection = msg.data
        self._last_detection_time_s = now_s
        self.targets.append(cx)
        if len(self.targets) > self._max_targets:
            self.targets.pop(0)

    def update_pose(self, linear, angular, dt):
        self.theta += angular * dt
        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt

    # =======================================================
    # FUNCIÓN DE CINEMÁTICA INVERSA (IK) SIMPLIFICADA
    # =======================================================
    def send_arm_to_height(self, target_z):
        z_clamped = max(0, min(target_z, 1000))
        pwm_hombro = int(110 + (z_clamped / 1000.0) * (510 - 110))
        pwm_codo = int(510 - (z_clamped / 1000.0) * (510 - 200))
        self.arm_pub.publish(String(data=f"ARM:SET:1,{pwm_hombro}"))
        self.arm_pub.publish(String(data=f"ARM:SET:2,{pwm_codo}"))

    def loop(self):
        now = self.get_clock().now()
        dt = (now - self._last_loop_time).nanoseconds / 1e9
        if dt <= 0.0:
            dt = 0.0
        self._last_loop_time = now

        msg = Twist()
        now_s = time.monotonic()
        image_center = max(float(self._image_width_px) / 2.0, 1.0)

        if (
            now_s - self._last_action_time_s > self._action_timeout_s
            and self.state in ["explore", "approach"]
        ):
            self.state = "explore"
            self.targets.clear()
            self.current_target = None
            self._last_action_time_s = now_s

        # --- MÁQUINA DE ESTADOS ---
        if self.state == "explore":
            msg.linear.x = 0.18
            msg.angular.z = 0.25

            if len(self.targets) >= self._target_lock_count:
                self.current_target = min(
                    self.targets, key=lambda x: abs(x - image_center)
                )
                self.state = "approach"
                self._last_action_time_s = now_s

        elif self.state == "approach":
            if now_s - self._last_detection_time_s > 1.5:
                self.state = "explore"
                return

            cx = self.current_target
            error = cx - image_center
            linear_speed = max(0.08, 0.25 - abs(error) / 200.0)

            if abs(error) < 15:
                msg.linear.x = linear_speed
                msg.angular.z = 0.0
                if now_s - self._last_detection_time_s > 1.8:
                    self.state = "collect"
                    self._last_action_time_s = now_s
            else:
                msg.linear.x = 0.0
                msg.angular.z = max(min(-error / 140.0, 0.5), -0.5)

        elif self.state == "collect":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            tiempo_recoleccion = now_s - self._last_action_time_s

            if tiempo_recoleccion < 0.5:
                self.arm_pub.publish(String(data="ARM:DEPLOY"))
            elif 2.0 < tiempo_recoleccion < 2.5:
                self.arm_pub.publish(String(data="ARM:STOW"))
            elif tiempo_recoleccion > 4.0:
                self.collected_rocks += 1
                self.get_logger().info(
                    f"Roca recolectada ({self.collected_rocks}/{self.max_rocks})"
                )
                self.targets.clear()
                self.current_target = None
                if self.collected_rocks >= self.max_rocks:
                    self.state = "return"
                else:
                    self.state = "explore"
                self._last_action_time_s = now_s

        elif self.state == "return":
            dx, dy = self.home_x - self.x, self.home_y - self.y
            dist = math.sqrt(dx**2 + dy**2)
            
            # Navegar ciegamente a odometría no basta por la arena.
            # Usar contenedor visual si está cerca (Regla de contenedor base).
            if dist <= 1.5 and self.inicio_cx is not None and now_s - self.last_inicio_time < 1.0:
                error = self.inicio_cx - 320
                if dist > 0.4:
                    msg.linear.x = 0.15
                    msg.angular.z = max(min(-error / 140.0, 0.4), -0.4)
                else:
                    self.arm_pub.publish(String(data="ARM:SET:1,800"))
                    self.state = "deposit"
                    self._last_action_time_s = now_s
            else:
                self.state = "deposit"
                self._last_action_time_s = now_s

        elif self.state == "deposit":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            tiempo_deposito = now_s - self._last_action_time_s

            if tiempo_deposito < 1.0:
                self.arm_pub.publish(String(data="ARM:SET:1,450"))
                self.arm_pub.publish(String(data="ARM:SET:2,200"))
            elif 3.0 < tiempo_deposito < 3.5:
                self.arm_pub.publish(String(data="ARM:SET:5,110"))
            elif tiempo_deposito > 5.0:
                self.get_logger().info("Depósito completado en Contenedor.")
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "finished"

        elif self.state == "evade":
            msg.linear.x = -0.20
            msg.angular.z = 0.6
            
            # Escapar por 2 segundos y luego volver a explorar
            if now_s - self._last_action_time_s > 2.0:
                self.state = "explore"
                self._last_action_time_s = now_s

        elif self.state == "mantenimiento":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

            if self.panel_height is None:
                if now_s - self._last_panel_search_log_s > 2.0:
                    self.get_logger().info("Buscando panel de mantenimiento...")
                    self._last_panel_search_log_s = now_s
                return

            t_mant = now_s - self._last_action_time_s
            base_z = self.panel_height

            if t_mant < 1.0:
                self.get_logger().info("Posicionando brazo para Interruptor 1...")
                self.send_arm_to_height(base_z + 50)
            elif 3.0 < t_mant < 3.5:
                self.get_logger().info("Posicionando brazo para Botón 1...")
                self.send_arm_to_height(base_z - 50)
            elif 5.0 < t_mant < 5.5:
                self.get_logger().info("Secuencia finalizada.")
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "explore" # O finished, según las reglas del torneo
                self._last_action_time_s = now_s
                
        elif self.state == "finished":
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        elif self.state == "emergency_stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if now_s - self._last_action_time_s > 0.5:
                self.arm_pub.publish(String(data="ARM:HOME"))

        if abs(msg.angular.z) > 0.5:
            msg.angular.z = 0.5 * (msg.angular.z / abs(msg.angular.z))

        self.update_pose(msg.linear.x, msg.angular.z, dt)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
