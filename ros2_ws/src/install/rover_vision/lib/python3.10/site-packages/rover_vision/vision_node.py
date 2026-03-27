import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.publisher_ = self.create_publisher(String, '/detections', 10)

        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, 320)
        self.cap.set(4, 240)

    def process_frame(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        kernel = np.ones((5,5), np.uint8)

        # ROJO
        lower_red1 = np.array([0,120,70])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([170,120,70])
        upper_red2 = np.array([180,255,255])

        mask_red = cv2.inRange(hsv, lower_red1, upper_red1) + \
                   cv2.inRange(hsv, lower_red2, upper_red2)

        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # AZUL
        lower_blue = np.array([94,80,2])
        upper_blue = np.array([126,255,255])

        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        # VERDE
        lower_green = np.array([25,52,72])
        upper_green = np.array([102,255,255])

        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)

        for color, mask in [("rojo", mask_red),
                            ("azul", mask_blue),
                            ("verde", mask_green)]:

            contours, _ = cv2.findContours(mask,
                                           cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)

                if area < 800:
                    continue

                x, y, w, h = cv2.boundingRect(cnt)
                cx = x + w // 2

                msg = String()
                msg.data = f"rock,{cx},{color}"
                self.publisher_.publish(msg)

                cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)

        # DETECCIÓN FIN
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresh,
                                       cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 5000:
                msg = String()
                msg.data = "fin,160,none"
                self.publisher_.publish(msg)

        cv2.imshow("Vision", frame)

        if cv2.waitKey(1) == 27:
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def run(self):
        while rclpy.ok():
            self.process_frame()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()
