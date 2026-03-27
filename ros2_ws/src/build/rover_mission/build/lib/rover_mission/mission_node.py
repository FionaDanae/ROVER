import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
import math


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        # SUBSCRIPCIONES
        self.subscription = self.create_subscription(
            String,
            '/detections',
            self.detection_callback,
            10
        )

        # PUBLICADOR
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # ESTADOS
        self.state = "explore"

        # DETECCIÓN
        self.last_detection = None
        self.last_time = time.time()
        self.detection_count = 0

        # OBJETIVOS
        self.targets = []
        self.current_target = None

        # INVENTARIO
        self.collected_rocks = 0
        self.max_rocks = 3

        # POSICIÓN ESTIMADA
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ORIGEN
        self.home_x = 0.0
        self.home_y = 0.0

        # CONTROL TIEMPO
        self.last_action_time = time.time()

        self.get_logger().info("Mission Node FINAL iniciado")

        # LOOP
        self.timer = self.create_timer(0.1, self.loop)

    # CALLBACK DETECCIÓN
    def detection_callback(self, msg):

        if not msg.data:
            return

        if msg.data == self.last_detection:
            self.detection_count += 1
        else:
            self.detection_count = 1

        self.last_detection = msg.data
        self.last_time = time.time()

        try:
            label, cx, color = msg.data.split(',')
            cx = int(cx)

            # DETECCIÓN FIN
            if label == "fin":
                self.get_logger().info("FIN detectado → regresar")
                self.state = "return"
                return

            # SOLO ROCAS
            if color == "rojo":
                self.targets.append(cx)

                if len(self.targets) > 10:
                    self.targets.pop(0)

        except:
            pass

    # ACTUALIZAR POSICIÓN
    def update_pose(self, linear, angular, dt=0.1):

        self.theta += angular * dt
        self.x += linear * math.cos(self.theta) * dt
        self.y += linear * math.sin(self.theta) * dt

    # LOOP PRINCIPAL
    def loop(self):

        msg = Twist()

        # TIMEOUT GLOBAL
        if time.time() - self.last_action_time > 12:
            self.get_logger().warn("Timeout global → explorar")
            self.state = "explore"
            self.targets.clear()
            self.current_target = None
            self.last_action_time = time.time()

        # EXPLORAR
        if self.state == "explore":

            msg.linear.x = 0.18
            msg.angular.z = 0.25

            # seleccionar objetivo
            if len(self.targets) > 5:
                self.current_target = min(
                    self.targets,
                    key=lambda x: abs(x - 160)
                )
                self.state = "approach"
                self.last_action_time = time.time()

        # ACERCARSE A ROCA
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

        # RECOLECTAR
        elif self.state == "collect":

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            if time.time() - self.last_action_time > 1.5:
                self.collected_rocks += 1
                self.get_logger().info(
                    f"Roca recolectada ({self.collected_rocks})"
                )

                self.targets.clear()
                self.current_target = None

                # si ya tiene suficientes → buscar FIN
                if self.collected_rocks >= self.max_rocks:
                    self.state = "explore"
                else:
                    self.state = "explore"

        # REGRESAR AL ORIGEN
        elif self.state == "return":

            dx = self.home_x - self.x
            dy = self.home_y - self.y

            distance = math.sqrt(dx**2 + dy**2)
            angle_to_home = math.atan2(dy, dx)

            angle_error = angle_to_home - self.theta

            # normalizar ángulo
            angle_error = math.atan2(
                math.sin(angle_error),
                math.cos(angle_error)
            )

            if distance > 0.2:

                msg.linear.x = 0.15
                msg.angular.z = max(
                    min(angle_error, 0.4),
                    -0.4
                )

            else:
                self.state = "deposit"
                self.last_action_time = time.time()

        # DEPOSITAR
        elif self.state == "deposit":

            msg.linear.x = 0.0
            msg.angular.z = 0.0

            if time.time() - self.last_action_time > 2.0:
                self.get_logger().info(
                    f"Depósito completado: {self.collected_rocks} rocas"
                )
                self.state = "finished"

        # FINAL
        elif self.state == "finished":

            msg.linear.x = 0.0
            msg.angular.z = 0.0

        # LIMITADOR
        if abs(msg.angular.z) > 0.5:
            msg.angular.z = 0.5 * (
                msg.angular.z / abs(msg.angular.z)
            )

        # actualizar odometría
        self.update_pose(msg.linear.x, msg.angular.z)

        # PUBLICAR
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
