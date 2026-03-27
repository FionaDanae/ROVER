#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.publisher_ = self.create_publisher(String, 'detections', 10)

        # Raspberry Pi Camera Module 3
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        # resolución optimizada
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

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ---------- ROJO ----------
        lower_red1 = np.array([0,100,100])
        upper_red1 = np.array([10,255,255])

        lower_red2 = np.array([160,100,100])
        upper_red2 = np.array([180,255,255])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2

        # ---------- AZUL ----------
        lower_blue = np.array([100,150,50])
        upper_blue = np.array([140,255,255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # ---------- VERDE ----------
        lower_green = np.array([40,70,70])
        upper_green = np.array([80,255,255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        self.detect_color(mask_red, frame, "rojo")
        self.detect_color(mask_blue, frame, "azul")
        self.detect_color(mask_green, frame, "verde")

        # mostrar solo si hay pantalla
        try:
            cv2.imshow("Vision", frame)
            cv2.waitKey(1)
        except:
            pass

    def detect_color(self, mask, frame, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)

            if area > 500:
                x, y, w, h = cv2.boundingRect(cnt)

                cx = int(x + w/2)

                msg = String()
                msg.data = f"rock,{cx},{color_name}"

                self.publisher_.publish(msg)

                self.get_logger().info(msg.data)

                cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
                cv2.putText(frame, color_name, (x,y-10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0,255,0), 2)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
