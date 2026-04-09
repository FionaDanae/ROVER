import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math
import time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class MappingNode(Node):

    def __init__(self):
        super().__init__('mapping_node')

        # detecciones de visión
        self.subscription = self.create_subscription(
            String,
            '/detections',
            self.detection_callback,
            10
        )

        # terreno
        self.sub_imu = self.create_subscription(
            String,
            '/terrain_status',
            self.terrain_cb,
            10
        )

        # LIDAR
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.last_scan = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.map_rocks = []
        self.map_terrain = []

        self.get_logger().info("Mapping Node iniciado - Exploración Científica")
        self.timer = self.create_timer(0.1, self.update_position)

    def scan_callback(self, msg):
        self.last_scan = msg

    def update_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )

            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y

            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.theta = math.atan2(siny_cosp, cosy_cosp)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

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

    def terrain_cb(self, msg):
        tipo_terreno = msg.data.lower()

        nuevo_terreno = {
            "x": round(self.x, 2),
            "y": round(self.y, 2),
            "tipo": tipo_terreno
        }

        is_new = True
        for t in self.map_terrain:
            if math.hypot(t["x"] - nuevo_terreno["x"], t["y"] - nuevo_terreno["y"]) < 0.5:
                is_new = False
                break

        if is_new:
            self.map_terrain.append(nuevo_terreno)
            self.get_logger().info(f"Accidente geográfico real registrado: {nuevo_terreno}")

    def detection_callback(self, msg):

        data = msg.data.split(',')

        if len(data) != 5:
            return

        label, cx, color, tamano, textura = data

        if label != "roca":
            return

        cx = int(cx)

        # cámara 640 px
        width = 640
        fov = 60.0

        angle = (cx - width/2) * (fov / width)
        angle_rad = math.radians(angle)

        # obtener distancia del LIDAR
        distance = self.get_lidar_distance(angle_rad)

        if distance is None:
            return

        # limitar distancia razonable
        if distance > 3.0:
            return

        rock_x = self.x + distance * math.cos(self.theta + angle_rad)
        rock_y = self.y + distance * math.sin(self.theta + angle_rad)

        rock = {
            "x": round(rock_x, 2),
            "y": round(rock_y, 2),
            "color": color,
            "tamano": tamano,
            "forma": "irregular",
            "textura": textura,
            "distance": round(distance, 2),
            "timestamp": time.time()
        }

        is_new = True
        for r in self.map_rocks:
            if math.hypot(r["x"] - rock["x"], r["y"] - rock["y"]) < 0.3:
                is_new = False
                break

        if is_new:
            self.map_rocks.append(rock)
            self.get_logger().info(f"Roca mapeada con LIDAR: {rock}")

    def destroy_node(self):
        with open("mapa_lunar_oficial.txt", "w") as f:
            f.write("--- INVENTARIO DE ROCAS ---\n")
            for rock in self.map_rocks:
                f.write(str(rock) + "\n")

            f.write("\n--- ACCIDENTES GEOGRAFICOS ---\n")
            for terr in self.map_terrain:
                f.write(str(terr) + "\n")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
