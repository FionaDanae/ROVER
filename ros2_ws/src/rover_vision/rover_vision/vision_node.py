#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import pytesseract

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(String, 'detections', 10)
        self.cap = cv2.VideoCapture(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
        texto_detectado = pytesseract.image_to_string(thresh, config='--psm 11').strip().upper()
        
        if "FIN" in texto_detectado:
            msg_fin = String()
            msg_fin.data = "fin,320,negro,N/A" 
            self.publisher_.publish(msg_fin)
            cv2.putText(frame, "LETRERO FIN ENCONTRADO", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_red1 = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(hsv, np.array([160,100,100]), np.array([180,255,255]))
        mask_red = mask_red1 + mask_red2

        mask_blue = cv2.inRange(hsv, np.array([100,150,50]), np.array([140,255,255]))
        mask_green = cv2.inRange(hsv, np.array([40,70,70]), np.array([80,255,255]))

        self.detect_color(mask_red, frame, "rojo")
        self.detect_color(mask_blue, frame, "azul")
        self.detect_color(mask_green, frame, "verde")

        try:
            cv2.imshow("Vision", frame)
            cv2.waitKey(1)
        except:
            pass

    def detect_color(self, mask, frame, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if not contours:
            return

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if area > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)
            cx = int(x + w/2)
            
            if area < 1500:
                tamano = "5cm3"
            elif area < 3000:
                tamano = "7cm3"
            elif area < 5000:
                tamano = "10cm3"
            else:
                tamano = "12cm3"

            msg = String()
            msg.data = f"roca,{cx},{color_name},{tamano}"
            self.publisher_.publish(msg)

            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            cv2.putText(frame, f"{color_name} {tamano}", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
