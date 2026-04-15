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
        self.terrain_sub = self.create_subscription(String, '/terrain_status', self.terrain_callback, 10)
        
        # Publicadores
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(String, '/arm_cmd', 10)

        # Variables de estado y lógica
        self.state = "explore"
        self.last_detection = None
        self.last_action_time = time.time()
        self.targets = []
        self.controles_detectados = []
        self.map_rocks = [] # Inicialización necesaria para evitar errores
        self.current_target = None
        self.front_distance = 999.0

        # ATENCIÓN: Necesitas declarar self.last_scan para que get_lidar_distance no falle
        self.last_scan = None 

        self.collected_rocks = 0
        self.max_rocks = 10

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.home_x = 0.0
        self.home_y = 0.0
        
        self.inicio_cx = None
        self.last_inicio_time = 0
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.mission_start = time.time()
        self.max_mission_time = 480 
        self.panel_height = None 

        self.get_logger().info("FAT RAT - Mission Node con Evasión de Obstáculos Iniciado")
        self.timer = self.create_timer(0.1, self.loop)

    def scan_callback(self, msg):
        """Procesa datos del LiDAR para detectar obstáculos frontales."""
        self.last_scan = msg # Guardamos el scan para la función get_lidar_distance
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
        data = msg.data.split(',')
        label = data[0]

        if label == "roca" and len(data) == 6:
            label, cx, color, tamano, textura, forma = data
        elif label == "roca" and len(data) == 3: # Si es una detección incompleta
            cx = int(data[1])
            color = data[2]
            return
        elif label == "fin" and self.state != "return" and self.state != "celebrate_fin":
            self.state = "celebrate_fin"
            self.last_action_time = time.time()
            self.get_logger().info("¡Letrero FIN detectado! Indicando visualmente (Regla 1.1).")
        elif label == "inicio":
            self.inicio_cx = int(data[1])
            self.last_inicio_time = time.time()
        elif label == "panel" and len(data) == 3:
            self.panel_height = int(data[2])
            # Forzar el inicio de la secuencia si estábamos explorando
            if self.state in ["explore", "approach"]:
                self.state = "mantenimiento"
                self.last_action_time = time.time()
                self.targets.clear() # Limpiar objetivos de rocas
                self.get_logger().info("¡Panel detectado! Iniciando secuencia de mantenimiento.")

        elif label == "control" and len(data) == 4:
            tipo_ctrl = data[1]
            cx_ctrl = int(data[2])
            estado_ctrl = data[3] 
            self.controles_detectados.append({"tipo": tipo_ctrl, "cx": cx_ctrl, "estado": estado_ctrl})

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
            
        self.targets.append(cx)

        self.map_rocks.append(rock)
        self.get_logger().info(f"Roca registrada: {rock}")

    def terrain_callback(self, msg):
        terreno = msg.data
        if terreno in ["surco", "pendiente"] and self.state == "explore":
            self.get_logger().warn(f"¡PELIGRO! {terreno.upper()} inminente. Iniciando maniobra evasiva.")
            self.state = "evade"
            self.last_action_time = time.time()

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
        now = time.time()

        if now - self.mission_start > self.max_mission_time:
            self.state = "return"
        
        if now - self.last_action_time > 12 and self.state in ["explore", "approach"]:
            self.state = "explore"
            self.targets.clear()
            self.current_target = None
            self.last_action_time = now

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
                self.last_action_time = now

        elif self.state == "approach":
            if now - self.last_action_time > 5.0:
                self.state = "explore"
                return

            cx = self.current_target
            error = cx - 320
            linear_speed = max(0.08, 0.25 - abs(error)/200)

            if abs(error) < 15:
                msg.linear.x = linear_speed
                msg.angular.z = 0.0
                if now - self.last_action_time > 1.8:
                    self.state = "collect"
                    self.last_action_time = now
            else:
                msg.linear.x = 0.0
                msg.angular.z = max(min(-error / 140.0, 0.5), -0.5)

        elif self.state == "collect":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            t_col = now - self.last_action_time
            if t_col < 0.5: 
                self.arm_pub.publish(String(data="ARM:DEPLOY"))
            elif 2.0 < t_col < 2.5: 
                self.arm_pub.publish(String(data="ARM:STOW"))
            elif t_col > 4.0:
                self.collected_rocks += 1
                self.targets.clear()
                self.state = "explore" 
                self.last_action_time = now

        elif self.state == "celebrate_fin":
            # Demostración requerida por Regla 1.1 "siempre mostrando o indicando que ya lo ha localizado"
            msg.linear.x = 0.0
            msg.angular.z = 0.8 # Giro de 3 segundos sobre su eje para indicar a los jueces
            if now - self.last_action_time > 3.0:
                self.state = "return"
                self.last_action_time = now
                self.get_logger().info("Regresando a base dictado por final de festejo.")

        elif self.state == "return":
            dx, dy = self.home_x - self.x, self.home_y - self.y
            dist = math.sqrt(dx**2 + dy**2)
            
            # Navegar ciegamente a odometría no basta por la arena.
            # Usar contenedor visual si está cerca (Regla de contenedor base).
            if dist <= 1.5 and hasattr(self, 'inicio_cx') and self.inicio_cx is not None and now - self.last_inicio_time < 1.0:
                error = self.inicio_cx - 320
                if dist > 0.4:
                    msg.linear.x = 0.15
                    msg.angular.z = max(min(-error / 140.0, 0.4), -0.4)
                else:
                    self.arm_pub.publish(String(data="ARM:SET:1,800"))
                    self.state = "deposit"
                    self.last_action_time = now
            else:
                error_a = math.atan2(dy, dx) - self.theta
                error_a = math.atan2(math.sin(error_a), math.cos(error_a))

                if dist > 0.4:
                    msg.linear.x = 0.15
                    msg.angular.z = max(min(error_a, 0.4), -0.4)
                else:
                    self.arm_pub.publish(String(data="ARM:SET:1,800"))
                    self.state = "deposit"
                    self.last_action_time = now

        elif self.state == "deposit":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            t_dep = now - self.last_action_time
            if t_dep < 1.0:
                self.arm_pub.publish(String(data="ARM:SET:1,800")) # Brazo arriba esquivando pared 20cm
                self.arm_pub.publish(String(data="ARM:SET:2,200"))
            elif 1.0 <= t_dep < 3.0:
                msg.linear.x = 0.15 # Avanzar 2 segundos
            elif 3.0 <= t_dep < 3.5:
                msg.linear.x = 0.0
                self.arm_pub.publish(String(data="ARM:SET:5,110")) # Abrir garra
            elif 3.5 <= t_dep < 4.5:
                msg.linear.x = -0.15 # Escapar
            elif t_dep > 5.0:
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "finished"

        elif self.state == "evade":
            msg.linear.x = -0.20
            msg.angular.z = 0.6
            
            # Escapar por 2 segundos y luego volver a explorar
            if now - self.last_action_time > 2.0:
                self.state = "explore"
                self.last_action_time = now

        elif self.state == "mantenimiento":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            t_mant = now - self.last_action_time
            
            # --- TIMEOUT DE RESCATE ---
            # Si lleva más de 12 segundos atascado en este estado sin avanzar
            if t_mant > 12.0 and (self.panel_height is None or len(self.controles_detectados) < 4):
                self.get_logger().warn("Timeout de Mantenimiento: No vi suficientes controles. Abortando cápsula.")
                self.state = "explore"
                self.last_action_time = now
                return

            # Esperar a tener el panel y al menos 4 controles detectados
            if self.panel_height is None or len(self.controles_detectados) < 4: 
                return

            bz = self.panel_height 

            # Filtramos para asegurar que tocamos 2 de cada uno como pide el PDF
            botones = [c for c in self.controles_detectados if c["tipo"] == "boton"][:2]
            switches = [c for c in self.controles_detectados if c["tipo"] == "interruptor"][:2]

            # Si después de filtrar faltan elementos, pero aún no hay timeout, seguimos esperando
            if len(botones) < 2 or len(switches) < 2:
                return

            # REINICIO DEL TIEMPO PARA LA SECUENCIA DE BRAZO
            # Esto es vital para que la secuencia empiece desde t=0 una vez que encontró todo
            if not hasattr(self, 'arm_sequence_started'):
                self.arm_sequence_started = now
                
            t_seq = now - self.arm_sequence_started

            # Aquí ya usas la altura dinámica bz basada en lo que vio la cámara
            if t_seq < 1.0: self.send_arm_to_height(bz) # Posición de espera frente al panel
            elif 3.0 < t_seq < 3.5: self.send_arm_to_height(bz + 20) # Simula tocar Botón 1
            elif 6.0 < t_seq < 6.5: self.send_arm_to_height(bz - 20) # Simula tocar Botón 2
            elif 9.0 < t_seq < 9.5: self.send_arm_to_height(bz + 40) # Simula tocar Switch 1
            elif 12.0 < t_seq < 12.5: self.send_arm_to_height(bz - 40) # Simula tocar Switch 2
            elif 15.0 < t_seq < 15.5:
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "explore" # O finished, según las reglas del torneo
                self.last_action_time = now
                del self.arm_sequence_started # Limpiamos para el futuro
                
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
