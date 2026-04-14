#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import numpy as np
import pytesseract
import threading

class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')
        self.publisher_ = self.create_publisher(String, 'detections', 10)
        self.cap = cv2.VideoCapture(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        self.frame_count = 0
        self.ocr_running = False

        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara")

        self.timer = self.create_timer(0.1, self.process_frame)

    def run_ocr(self, img_thresh):
        try:
            texto_detectado = pytesseract.image_to_string(img_thresh, config='--psm 11').strip().upper()
            if "FIN" in texto_detectado:
                msg_fin = String()
                msg_fin.data = "fin,320,negro,N/A" 
                self.publisher_.publish(msg_fin)
        except Exception:
            pass 
        finally:
            self.ocr_running = False

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1

        # =======================================================
        # OCR (Letrero FIN) - Se ejecuta asíncronamente
        # =======================================================
        if self.frame_count % 15 == 0 and not self.ocr_running:
            self.ocr_running = True
            roi_top = frame[0:240, 0:640] 
            gray = cv2.cvtColor(roi_top, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)
            
            threading.Thread(target=self.run_ocr, args=(thresh,), daemon=True).start()

        # =======================================================
        # DETECCIÓN DE ROCAS POR COLOR
        # =======================================================
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_red1 = cv2.inRange(hsv, np.array([0,100,100]), np.array([10,255,255]))
        mask_red2 = cv2.inRange(hsv, np.array([160,100,100]), np.array([180,255,255]))
        mask_red = mask_red1 + mask_red2

        mask_blue = cv2.inRange(hsv, np.array([100,150,50]), np.array([140,255,255]))
        mask_green = cv2.inRange(hsv, np.array([40,70,70]), np.array([80,255,255]))

        self.detect_color(mask_red, frame, "rojo")
        self.detect_color(mask_blue, frame, "azul")
        self.detect_color(mask_green, frame, "verde")

        # =======================================================
        # DETECCIÓN DEL PANEL DE MANTENIMIENTO
        # =======================================================
        mask_gray = cv2.inRange(hsv, np.array([0, 0, 50]), np.array([180, 50, 200]))
        contours_gray, _ = cv2.findContours(mask_gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours_gray:
            largest_gray = max(contours_gray, key=cv2.contourArea)
            if cv2.contourArea(largest_gray) > 5000:
                x, y, w, h = cv2.boundingRect(largest_gray)
                cx = int(x + w/2)
                cy = int(y + h/2)
                
                altura_estimada_mm = int(1000 - (y * 2)) 
                
                msg_panel = String()
                msg_panel.data = f"panel,{cx},{altura_estimada_mm}"
                self.publisher_.publish(msg_panel)
                
                cv2.rectangle(frame, (x,y), (x+w,y+h), (255,0,255), 2)
                cv2.putText(frame, f"PANEL: {altura_estimada_mm}mm", (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,255), 2)

                # =======================================================
                # DETECCIÓN DE CONTROLES (Dentro del área del Panel)
                # =======================================================
                panel_roi = frame[y:y+h, x:x+w]
                
                if panel_roi.size > 0:
                    gray_panel = cv2.cvtColor(panel_roi, cv2.COLOR_BGR2GRAY)
                    blur = cv2.GaussianBlur(gray_panel, (5,5), 0)
              
                    # --- DETECCIÓN DE BOTONES ---
                    circles = cv2.HoughCircles(
                        blur,
                        cv2.HOUGH_GRADIENT,
                        dp=1.2,
                        minDist=40,
                        param1=50,
                        param2=30,
                        minRadius=10,
                        maxRadius=50
                    )

                    if circles is not None:
                        circles = np.uint16(np.around(circles))
                        for c in circles[0, :2]:
                            cx_btn = int(x + c[0])
                            cy_btn = int(y + c[1])
                            r = c[2]
                            
                            roi_btn = gray_panel[max(c[1]-r,0):c[1]+r, max(c[0]-r,0):c[0]+r]
                            estado = "on" if np.mean(roi_btn) > 150 else "off"

                            msg = String()
                            msg.data = f"control,boton,{cx_btn},{estado}"
                            self.publisher_.publish(msg)

                            cv2.circle(frame, (cx_btn, cy_btn), r, (0,255,255), 2)
                            cv2.putText(frame, f"BTN {estado}", (cx_btn, cy_btn), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 2)

                    # --- DETECCIÓN DE INTERRUPTORES ---
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
                            cx_sw = int(x + x2 + w2/2)
                            estado = "on" if h2 > w2 else "off"

                            msg = String()
                            msg.data = f"control,interruptor,{cx_sw},{estado}"
                            self.publisher_.publish(msg)

                            cv2.rectangle(frame, (x + x2, y + y2), (x + x2 + w2, y + y2 + h2), (255,255,0), 2)
                            cv2.putText(frame, f"SW {estado}", (x + x2, y + y2 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
                            switches_detected += 1

                        if switches_detected >= 2:
                            break

        # =======================================================
        # DETECCIÓN DE CONTENEDOR BLANCO (INICIO)
        # =======================================================
        mask_white = cv2.inRange(hsv, np.array([0, 0, 200]), np.array([180, 50, 255]))
        contours_white, _ = cv2.findContours(mask_white, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours_white:
            largest_white = max(contours_white, key=cv2.contourArea)
            if cv2.contourArea(largest_white) > 3000:
                x_w, y_w, w_w, h_w = cv2.boundingRect(largest_white)
                cx_w = int(x_w + w_w/2)
                
                msg_inicio = String()
                msg_inicio.data = f"inicio,{cx_w}"
                self.publisher_.publish(msg_inicio)
                
                cv2.rectangle(frame, (x_w,y_w), (x_w+w_w,y_w+h_w), (255,255,255), 2)
                cv2.putText(frame, "INICIO", (x_w,y_w-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2)

        # Mostrar imagen
        try:
            cv2.imshow("Vision - Fat Rat", frame)
            cv2.waitKey(1)
        except Exception:
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
            cy = int(y + h/2)

            # ============================
            # TAMAÑO
            # ============================
            if area < 1500: tamano = "5cm3"
            elif area < 3000: tamano = "7cm3"
            elif area < 5000: tamano = "10cm3"
            else: tamano = "12cm3"

            # ============================
            # TEXTURA
            # ============================
            roi_color = frame[y:y+h, x:x+w]

            if roi_color.size > 0:
                roi_gray = cv2.cvtColor(roi_color, cv2.COLOR_BGR2GRAY)
                varianza = cv2.Laplacian(roi_gray, cv2.CV_64F).var()
                textura = "rugosa" if varianza > 300 else "lisa"
            else:
                textura = "no_determinada"

            # ============================
            # FORMA
            # ============================
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

            # ============================
            # PUBLICACIÓN
            # ============================
            msg = String()
            msg.data = f"roca,{cx},{cy},{color_name},{tamano},{textura},{forma}"
            self.publisher_.publish(msg)

            # Visualización
            cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
            cv2.putText(frame, f"{color_name} {forma}", (x,y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

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

if __name__ == '__main__':
    main()
