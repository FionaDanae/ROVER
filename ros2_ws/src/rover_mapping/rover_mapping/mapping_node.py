import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time

class MappingNode(Node):

    def __init__(self):
        super().__init__('mapping_node')

        self.subscription = self.create_subscription(String, '/detections', self.detection_callback, 10)
        self.sub_imu = self.create_subscription(String, '/terrain_status', self.terrain_cb, 10)
        
        # ODOMETRÍA REAL 
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

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

    def cmd_cb(self, msg):
        self.current_v = msg.linear.x
        self.current_w = msg.angular.z

    def update_position(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        self.x += self.current_v * math.cos(self.theta) * dt
        self.y += self.current_v * math.sin(self.theta) * dt
        self.theta += self.current_w * dt

    def terrain_cb(self, msg):
        tipo_terreno = msg.data.lower()
        nuevo_terreno = {"x": round(self.x, 2), "y": round(self.y, 2), "tipo": tipo_terreno}
        
        is_new = True
        for t in self.map_terrain:
            if math.hypot(t["x"] - nuevo_terreno["x"], t["y"] - nuevo_terreno["y"]) < 0.5:
                is_new = False
                break
                
        if is_new:
            self.map_terrain.append(nuevo_terreno)
            self.get_logger().info(f"Accidente geográfico: {nuevo_terreno}")

    def detection_callback(self, msg):
        data = msg.data.split(',')
        if len(data) != 4: 
            return

        label, cx, color, tamano = data
        if label != "roca":
            return

        cx = int(cx)
        width = 640 
        fov = 60  
        angle = (cx - width/2) * (fov / width)
        angle_rad = math.radians(angle)

        distance = 1.0 
        rock_x = self.x + distance * math.cos(self.theta + angle_rad)
        rock_y = self.y + distance * math.sin(self.theta + angle_rad)

        rock = {"x": round(rock_x, 2), "y": round(rock_y, 2), "color": color, "tamano": tamano, "forma": "irregular"}

        is_new = True
        for r in self.map_rocks:
            if math.hypot(r["x"] - rock["x"], r["y"] - rock["y"]) < 0.3:
                is_new = False
                break
                
        if is_new:
            self.map_rocks.append(rock)
            self.get_logger().info(f"Roca: {rock}")

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
