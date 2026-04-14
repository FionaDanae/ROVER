import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
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

        # Peso real
        self.weight_sub = self.create_subscription(
            String,
            '/rock_weight',
            self.weight_callback,
            10
        )

        self.last_scan = None

        # Publicadores visuales RViz2
        self.rock_marker_pub = self.create_publisher(MarkerArray, '/rock_markers', 10)
        self.terrain_marker_pub = self.create_publisher(MarkerArray, '/terrain_markers', 10)

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
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

    def weight_callback(self, msg):
        try:
            peso = float(msg.data)
            # Buscar la roca más cercana al rover (porque estamos sobre ella)
            closest_rock = None
            min_dist = 0.5 # 50 cm de tolerancia
            for r in self.map_rocks:
                dist = math.hypot(r["x"] - self.x, r["y"] - self.y)
                if dist < min_dist:
                    min_dist = dist
                    closest_rock = r
            
            if closest_rock:
                closest_rock["peso"] = peso
                self.get_logger().info(f"Asignado peso de {peso}g a la roca cerca de X:{closest_rock['x']}, Y:{closest_rock['y']}")
        except ValueError:
            pass

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

        if len(data) != 7:
            return

        label, cx, cy, color, tamano, textura, forma = data

        if label != "roca":
            return

        cx, cy = int(cx), int(cy)

        # cámara 640 px
        width = 640
        fov = 60.0

        angle = (cx - width/2) * (fov / width)
        angle_rad = math.radians(angle)

        # Calibración cámara
        H_c = 0.25  # Altura de tu cámara desde el suelo en metros (MÍDELO Y AJÚSTALO)
        theta_c = math.radians(15.0)  # Ángulo de inclinación hacia el piso (MÍDELO)
        fov_v = math.radians(45.0) # FOV Vertical aproximado de una Logitech (AJÚSTALO)
        h_px = 480.0 # Resolución en Y

        # Cálculo matemático de la distancia (Modelo Pinhole)
        alpha = (cy - (h_px / 2.0)) * (fov_v / h_px)
        
        try:
            distance = H_c / math.tan(theta_c + alpha)
        except ZeroDivisionError:
            return # Evitar errores si la cámara apunta directo al horizonte

        # Si la distancia es ilógica (muy lejos o negativa), ignorar
        if distance < 0.1 or distance > 3.0:
            return

        # Cálculo del ángulo horizontal para el mapa
        width = 640.0
        fov_h = math.radians(60.0)
        angle_rad = (cx - width/2) * (fov_h / width)

        rock_x = self.x + distance * math.cos(self.theta - angle_rad)
        rock_y = self.y + distance * math.sin(self.theta - angle_rad)

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

    def get_color_marker(self, color_name):
        c = [1.0, 1.0, 1.0] # Default white
        if color_name == "rojo": c = [1.0, 0.0, 0.0]
        elif color_name == "azul": c = [0.0, 0.0, 1.0]
        elif color_name == "verde": c = [0.0, 1.0, 0.0]
        return c

    def publish_markers(self):
        rock_array = MarkerArray()
        for i, r in enumerate(self.map_rocks):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "rocks"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = r["x"]
            m.pose.position.y = r["y"]
            m.pose.position.z = 0.05 # Ligeramente sobre la arena
            m.pose.orientation.w = 1.0
            
            # Escala basada en tamaño si quisiéramos, por defecto 5cm
            escala = 0.05
            if r["tamano"] == "12cm3": escala = 0.12
            elif r["tamano"] == "10cm3": escala = 0.10
            elif r["tamano"] == "7cm3": escala = 0.07

            m.scale.x = escala
            m.scale.y = escala
            m.scale.z = escala
            
            rgbs = self.get_color_marker(r["color"])
            m.color.r = rgbs[0]
            m.color.g = rgbs[1]
            m.color.b = rgbs[2]
            m.color.a = 1.0
            rock_array.markers.append(m)
            
        self.rock_marker_pub.publish(rock_array)

        terr_array = MarkerArray()
        for i, t in enumerate(self.map_terrain):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "terrain"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = t["x"]
            m.pose.position.y = t["y"]
            m.pose.position.z = 0.02
            m.pose.orientation.w = 1.0
            m.scale.x = 0.4
            m.scale.y = 0.4
            m.scale.z = 0.05
            
            # Color por accidente (Amarillo: Surco, Morado: Pendiente, Cyan: Valle)
            if t["tipo"] == "surco":
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0 
            elif t["tipo"] == "pendiente":
                m.color.r, m.color.g, m.color.b = 0.8, 0.0, 0.8
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 1.0
            
            m.color.a = 0.6  # Translúcido
            terr_array.markers.append(m)
            
        self.terrain_marker_pub.publish(terr_array)

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
