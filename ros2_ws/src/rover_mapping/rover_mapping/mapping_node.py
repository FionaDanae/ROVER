import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import math
import time

class MappingNode(Node):

    def __init__(self):
        super().__init__('mapping_node')

        self.subscription = self.create_subscription(
            String,
            '/detections',
            self.detection_callback,
            10
        )

        # Posición del robot (estimada)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()

        self.map = []

        self.get_logger().info("Mapping Node iniciado")

        # Timer para simular movimiento
        self.timer = self.create_timer(0.1, self.update_position)

    def update_position(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # SIMULACIÓN (luego se conecta a /cmd_vel real)
        v = 0.2   # velocidad lineal
        w = 0.0   # velocidad angular

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

    def detection_callback(self, msg):

        data = msg.data.split(',')

        if len(data) != 3:
            return

        _, cx, color = data
        cx = int(cx)

        # CONVERTIR PIXEL → ANGULO
        width = 320
        fov = 60  # grados cámara

        angle = (cx - width/2) * (fov / width)
        angle_rad = math.radians(angle)

        # ESTIMAR DISTANCIA
        distance = 1.0

        # COORDENADAS GLOBALES
        rock_x = self.x + distance * math.cos(self.theta + angle_rad)
        rock_y = self.y + distance * math.sin(self.theta + angle_rad)

        rock = {
            "x": round(rock_x, 2),
            "y": round(rock_y, 2),
            "color": color
        }

        self.map.append(rock)

        self.get_logger().info(f"Roca detectada: {rock}")

    def destroy_node(self):
        # Guardar mapa al cerrar
        with open("map.txt", "w") as f:
            for rock in self.map:
                f.write(str(rock) + "\n")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
