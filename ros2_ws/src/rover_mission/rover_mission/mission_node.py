import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import math
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

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
        self.targets = []
        self.current_target = None

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
        self.max_mission_time = 480 #número de segundos
        
        # Variables para el mantenimiento dinámico
        self.panel_height = None 

        self.get_logger().info("FAT RAT - Mission Node Iniciado (Modo 100% Autónomo)")
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
                    
            if "panel" in label.lower():
                # La visión pasa la altura aproximada (ej. panel,320,800)
                self.panel_height = int(data_parts[2])

        except Exception as e:
            pass

    def update_pose(self, linear, angular, dt=0.1):
        self.theta += angular * dt
        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt
        
    def update_pose(self, linear, angular, dt=0.1):
        try:
            self.theta += angular * dt
            
            self.x += trans.transform.translation.x
            self.y += trans.transform.translation.y
            
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 -2 * (q.y * q.y + q.z * q.z)
            self.theta = math.atan2(siny_cosp, cosy_cosp)
            
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    # =======================================================
    # FUNCIÓN DE CINEMÁTICA INVERSA (IK) SIMPLIFICADA
    # =======================================================
    def send_arm_to_height(self, target_z):
        """
        Mapea una altura objetivo en Z (mm) a valores PWM para los servos.
        NOTA: Deberás ajustar los rangos (out_min, out_max) según la longitud
        física de los eslabones del FAT RAT.
        """
        # Restringimos la altura operativa entre 0 y 1000 mm según reglamento
        z_clamped = max(0, min(target_z, 1000))
        
        # Ejemplo de interpolación lineal para el hombro (Canal 1)
        # Asumiendo que 110 PWM es altura mínima y 510 PWM es altura máxima
        pwm_hombro = int(110 + (z_clamped / 1000.0) * (510 - 110))
        
        # Ejemplo para el codo (Canal 2), compensando el movimiento del hombro
        pwm_codo = int(510 - (z_clamped / 1000.0) * (510 - 200))
        
        # Mandamos los comandos SET que la ESP32 sí reconoce
        self.arm_pub.publish(String(data=f"ARM:SET:1,{pwm_hombro}"))
        self.arm_pub.publish(String(data=f"ARM:SET:2,{pwm_codo}"))
        
        self.get_logger().info(f"IK Calculado para Z={target_z}mm -> Hombro:{pwm_hombro}, Codo:{pwm_codo}")

    def loop(self):
        msg = Twist()

        if time.time() - self.mission_start > self.max_mission_time:
            self.get_logger().info("de regreso pa, se le acabó el tiempo")
            self.state = "return"
        
        if time.time() - self.last_action_time > 12 and self.state in ["explore", "approach"]:
            self.state = "explore"
            self.targets.clear()
            self.current_target = None
            self.last_action_time = time.time()

        if self.state == "explore":
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
            tiempo_recoleccion = time.time() - self.last_action_time
            
            if tiempo_recoleccion < 0.5:
                self.arm_pub.publish(String(data="ARM:DEPLOY"))
            elif 2.0 < tiempo_recoleccion < 2.5:
                self.arm_pub.publish(String(data="ARM:STOW"))
            elif tiempo_recoleccion > 4.0:
                self.collected_rocks += 1
                self.get_logger().info(f"Roca recolectada ({self.collected_rocks}/{self.max_rocks})")
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
                self.get_logger().info(f"Depósito completado en Contenedor.")
                self.arm_pub.publish(String(data="ARM:HOME"))
                self.state = "finished"

        elif self.state == "mantenimiento":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            if self.panel_height is None:
                self.get_logger().info("Buscando panel de mantenimiento...", throttle_duration_sec=2)
                return

            t_mant = time.time() - self.last_action_time
            base_z = self.panel_height 
            
            # Secuencia
            if t_mant < 1.0:
                self.get_logger().info("Posicionando brazo para Interruptor 1...")
                self.send_arm_to_height(base_z + 50) 
            elif 3.0 < t_mant < 3.5:
                self.get_logger().info("Posicionando brazo para Botón 1...")
                self.send_arm_to_height(base_z - 20) 
            elif 6.0 < t_mant < 6.5:
                self.get_logger().info("Posicionando brazo para Interruptor 2...")
                # Agregamos un offset diferente para simular alcanzar otro interruptor
                self.send_arm_to_height(base_z + 80) 
            elif 9.0 < t_mant < 9.5:
                self.get_logger().info("Posicionando brazo para Botón 2...")
                # Agregamos un offset diferente para simular alcanzar otro botón
                self.send_arm_to_height(base_z - 50) 
            elif 12.0 < t_mant < 12.5:
                self.get_logger().info("Secuencia finalizada. Activación de cápsula completa.")
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
