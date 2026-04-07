import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class MappingNode(Node):

    def __init__(self):
        super().__init__('mapping_node')

        self.subscription = self.create_subscription(String, '/detections', self.callback, 10)
        self.sub_imu = self.create_subscription(String, '/terrain_status', self.terrain_cb, 10)
        
        # Configuración de TF2 (Para escuchar la posición desde SLAM)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.current_v = 0.0
        self.current_w = 0.0

        self.last_time = time.time()
        self.map_rocks = []
        self.map_terrain = []

        self.get_logger().info("Mapping Node iniciado - Exploración Científica")
        self.timer = self.create_timer(0.1, self.update_position)

    def update_position(self):
        # En lugar de calcular (v * dt), le preguntamos a ROS exactamente dónde estamos
        try:
            # Buscamos la transformación desde el 'map' hasta el 'base_link' (rover)
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            
            # Extraer X y Y reales
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y
            
            # Extraer rotación (Cuaternión a Ángulo Euler Z / Theta)
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.theta = math.atan2(siny_cosp, cosy_cosp)

        except (LookupException, ConnectivityException, ExtrapolationException):
            # Si el SLAM aún no arranca o no hay mapa, ignoramos esta iteración
            pass

    def terrain_cb(self, msg):
        # Este callback ahora debe recibir datos reales de un IMU o de la cámara de profundidad.
        tipo_terreno = msg.data.lower()
        nuevo_terreno = {"x": round(self.x, 2), "y": round(self.y, 2), "tipo": tipo_terreno}
        
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
        width = 640 
        fov = 60  
        angle = (cx - width/2) * (fov / width)
        angle_rad = math.radians(angle)

        distance = 0.8 
        rock_x = self.x + distance * math.cos(self.theta + angle_rad)
        rock_y = self.y + distance * math.sin(self.theta + angle_rad)

        rock = {
            "x": round(rock_x, 2), 
            "y": round(rock_y, 2), 
            "color": color, 
            "tamano": tamano, 
            "forma": "irregular",
            "textura": textura,
            "timestamp": time.time() 
        }

        is_new = True
        for r in self.map_rocks:
            if math.hypot(r["x"] - rock["x"], r["y"] - rock["y"]) < 0.3:
                is_new = False
                break
                
        if is_new:
            self.map_rocks.append(rock)
            self.get_logger().info(f"Roca mapeada: {rock}")

	if "panel" in label.lower():
            self.state = "mantenimiento"
            self.last_action_time = time.time()

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
