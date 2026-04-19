#!/usr/bin/env python3

import threading

import cv2
import numpy as np
import pytesseract
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("camera_index", 0)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("fps", 30)
        self.declare_parameter("timer_period_s", 0.1)
        self.declare_parameter("enable_gui", False)
        self.declare_parameter("enable_people_detection", True)
        self.declare_parameter("enable_ocr", True)
        self.declare_parameter("ocr_every_n_frames", 15)
        self.declare_parameter("enable_panel_detection", True)

        self._detections_topic = self.get_parameter("detections_topic").value
        self._camera_index = int(self.get_parameter("camera_index").value)
        self._frame_width = int(self.get_parameter("frame_width").value)
        self._frame_height = int(self.get_parameter("frame_height").value)
        self._fps = int(self.get_parameter("fps").value)
        self._timer_period_s = float(self.get_parameter("timer_period_s").value)
        self._enable_gui = bool(self.get_parameter("enable_gui").value)
        self._enable_people_detection = bool(self.get_parameter("enable_people_detection").value)
        self._enable_ocr = bool(self.get_parameter("enable_ocr").value)
        self._ocr_every_n_frames = int(self.get_parameter("ocr_every_n_frames").value)
        self._enable_panel_detection = bool(self.get_parameter("enable_panel_detection").value)

        self.publisher_ = self.create_publisher(String, self._detections_topic, 10)
        self.cap = cv2.VideoCapture(self._camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(self._frame_width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self._frame_height))
        self.cap.set(cv2.CAP_PROP_FPS, float(self._fps))

        self.frame_count = 0
        self.ocr_running = False
        self._ocr_lock = threading.Lock()

        self.hog = None
        if self._enable_people_detection:
            try:
                self.hog = cv2.HOGDescriptor()
                self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
            except Exception as exc:
                self.get_logger().warning(f"No se pudo inicializar detector de personas: {exc}")
                self.hog = None

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la camara")

        self.timer = self.create_timer(max(self._timer_period_s, 0.02), self.process_frame)

    def run_ocr(self, img_thresh):
        try:
            texto_detectado = pytesseract.image_to_string(img_thresh, config="--psm 11").strip().upper()
            if "FIN" in texto_detectado:
                self.publisher_.publish(String(data="fin,320,negro,N/A"))
        except Exception:
            pass
        finally:
            with self._ocr_lock:
                self.ocr_running = False

    def process_frame(self):
        if self.cap is None:
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1

        if self._enable_people_detection and self.hog is not None:
            boxes, _weights = self.hog.detectMultiScale(
                frame,
                winStride=(8, 8),
                padding=(4, 4),
                scale=1.05,
            )
            if len(boxes) > 0:
                self.publisher_.publish(String(data="persona,stop_all"))
                cv2.putText(
                    frame,
                    "PERSONA DETECTADA - STOP ALL",
                    (50, 100),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    3,
                )

        if self._enable_ocr and self._ocr_every_n_frames > 0:
            start_ocr = False
            if self.frame_count % self._ocr_every_n_frames == 0:
                with self._ocr_lock:
                    if not self.ocr_running:
                        self.ocr_running = True
                        start_ocr = True

            if start_ocr:
                roi_h = min(240, frame.shape[0])
                roi_w = min(640, frame.shape[1])
                roi_top = frame[0:roi_h, 0:roi_w]
                gray = cv2.cvtColor(roi_top, cv2.COLOR_BGR2GRAY)
                _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
                threading.Thread(target=self.run_ocr, args=(thresh,), daemon=True).start()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_red1 = cv2.inRange(hsv, np.array([0, 100, 100]), np.array([10, 255, 255]))
        mask_red2 = cv2.inRange(hsv, np.array([160, 100, 100]), np.array([180, 255, 255]))
        mask_red = mask_red1 + mask_red2
        mask_blue = cv2.inRange(hsv, np.array([100, 150, 50]), np.array([140, 255, 255]))
        mask_green = cv2.inRange(hsv, np.array([40, 70, 70]), np.array([80, 255, 255]))

        self.detect_color(mask_red, frame, "rojo")
        self.detect_color(mask_blue, frame, "azul")
        self.detect_color(mask_green, frame, "verde")

        if self._enable_panel_detection:
            mask_gray = cv2.inRange(hsv, np.array([0, 0, 50]), np.array([180, 50, 200]))
            contours_gray, _ = cv2.findContours(mask_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours_gray:
                largest_gray = max(contours_gray, key=cv2.contourArea)
                if cv2.contourArea(largest_gray) > 5000:
                    x, y, w, h = cv2.boundingRect(largest_gray)
                    cx = int(x + w / 2)
                    altura_estimada_mm = int(1000 - (y * 2))

                    self.publisher_.publish(String(data=f"panel,{cx},{altura_estimada_mm}"))
                    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 255), 2)
                    cv2.putText(
                        frame,
                        f"PANEL: {altura_estimada_mm}mm",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 0, 255),
                        2,
                    )

                    panel_roi = frame[y : y + h, x : x + w]
                    if panel_roi.size > 0:
                        gray_panel = cv2.cvtColor(panel_roi, cv2.COLOR_BGR2GRAY)
                        blur = cv2.GaussianBlur(gray_panel, (5, 5), 0)

                        circles = cv2.HoughCircles(
                            blur,
                            cv2.HOUGH_GRADIENT,
                            dp=1.2,
                            minDist=40,
                            param1=50,
                            param2=30,
                            minRadius=10,
                            maxRadius=50,
                        )
                        if circles is not None:
                            circles = np.uint16(np.around(circles))
                            for c in circles[0, :2]:
                                cx_btn = int(x + c[0])
                                cy_btn = int(y + c[1])
                                r = int(c[2])
                                roi_btn = gray_panel[max(c[1] - r, 0) : c[1] + r, max(c[0] - r, 0) : c[0] + r]
                                estado = "on" if roi_btn.size > 0 and np.mean(roi_btn) > 150 else "off"
                                self.publisher_.publish(String(data=f"control,boton,{cx_btn},{estado}"))
                                cv2.circle(frame, (cx_btn, cy_btn), r, (0, 255, 255), 2)

                        edges = cv2.Canny(blur, 50, 150)
                        contours_ctrl, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                        switches_detected = 0
                        for cnt in contours_ctrl:
                            area = cv2.contourArea(cnt)
                            if area < 300:
                                continue
                            x2, y2, w2, h2 = cv2.boundingRect(cnt)
                            aspect_ratio = h2 / float(w2) if w2 > 0 else 0
                            if aspect_ratio > 1.5 or aspect_ratio < 0.5:
                                cx_sw = int(x + x2 + w2 / 2)
                                estado = "on" if h2 > w2 else "off"
                                self.publisher_.publish(String(data=f"control,interruptor,{cx_sw},{estado}"))
                                cv2.rectangle(
                                    frame, (x + x2, y + y2), (x + x2 + w2, y + y2 + h2), (255, 255, 0), 2
                                )
                                switches_detected += 1
                            if switches_detected >= 2:
                                break

        mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours_white:
            largest_white = max(contours_white, key=cv2.contourArea)
            if cv2.contourArea(largest_white) > 3000:
                x_w, y_w, w_w, h_w = cv2.boundingRect(largest_white)
                cx_w = int(x_w + w_w / 2)
                self.publisher_.publish(String(data=f"inicio,{cx_w}"))
                cv2.rectangle(frame, (x_w, y_w), (x_w + w_w, y_w + h_w), (255, 255, 255), 2)

        if self._enable_gui:
            try:
                cv2.imshow("Vision - Fat Rat", frame)
                cv2.waitKey(1)
            except Exception:
                self._enable_gui = False

    def detect_color(self, mask, frame, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area <= 500:
            return

        x, y, w, h = cv2.boundingRect(largest_contour)
        cx = int(x + w / 2)
        cy = int(y + h / 2)

        if area < 1500:
            tamano = "5cm3"
        elif area < 3000:
            tamano = "7cm3"
        elif area < 5000:
            tamano = "10cm3"
        else:
            tamano = "12cm3"

        roi_color = frame[y : y + h, x : x + w]
        if roi_color.size > 0:
            roi_gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
            varianza = cv2.Laplacian(roi_gray, cv2.CV_64F).var()
            textura = "rugosa" if varianza > 300 else "lisa"
        else:
            textura = "no_determinada"

        peri = cv2.arcLength(largest_contour, True)
        approx = cv2.approxPolyDP(largest_contour, 0.04 * peri, True)
        num_vertices = len(approx)
        aspect_ratio = w / float(h) if h > 0 else 0

        if num_vertices >= 8:
            forma = "esferica"
        elif 4 <= num_vertices <= 6:
            forma = "cubica"
        elif aspect_ratio > 1.5 or aspect_ratio < 0.67:
            forma = "alargada"
        else:
            forma = "irregular"

        self.publisher_.publish(String(data=f"roca,{cx},{cy},{color_name},{tamano},{textura},{forma}"))
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(frame, f"{color_name} {forma}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        if self._enable_gui:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
