#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pathlib import Path
from ultralytics import YOLO

class Detection:
    def __init__(self):
        rospy.init_node('vision_node', anonymous=True)
        self.rate = rospy.Rate(20)

        # Membaca parameter dari ROS Parameter Server
        self.max_yaw = rospy.get_param('~max_yaw', 1700)
        self.min_yaw = rospy.get_param('~min_yaw', 1300)
        self.initial_yaw = 1500  # Titik tengah yaw, tetap sama
        self.WIDTH = rospy.get_param('~camera_width', 640)
        self.HEIGHT = rospy.get_param('~camera_height', 480)
        self.camera_device = rospy.get_param('~camera_device', 0)  # Default kamera pertama

        # Publisher untuk mengirim gambar dengan hasil deteksi
        self.navigation_vision_pub = rospy.Publisher('/robot/vision/navigation', Image, queue_size=1)

        # Load model YOLO dari file .pt
        ROOT = Path(__file__).resolve().parents[0]
        self.model = YOLO(model=f'{ROOT}/model/best.pt', task='detect')

        # Variabel tambahan
        self.bridge = CvBridge()
        self.fps = 0
        self.prev_time = rospy.get_time()

    def draw_degree_lines(self, frame):
        mid_y = self.HEIGHT // 2
        cv2.line(frame, (0, mid_y), (self.WIDTH, mid_y), (255, 0, 0), 2)

        for degree in range(0, 91, 5):
            x = int((degree / 90) * self.WIDTH)
            cv2.line(frame, (x, mid_y-10), (x, mid_y+10), (0, 255, 0), 2)

        return frame

    def calculate_yaw(self, midpoint_x, boat_type):
        normalized_pos = midpoint_x / self.WIDTH
        frame_midpoint = self.WIDTH // 2
        
        yaw_range = self.max_yaw - 1500  # Yaw range dihitung dari 1500 sebagai titik tengah

        yaw, direction = self.initial_yaw, "Tidak diketahui"

        if boat_type == 'boat_right':
            yaw = int(1500 - (normalized_pos * yaw_range))  # Penurunan yaw ke kiri
            direction = "Ke kiri"
        elif boat_type == 'boat_left':
            yaw = int(1500 + (normalized_pos * yaw_range))  # Penambahan yaw ke kanan
            direction = "Ke kanan"
        elif boat_type in ['boat_front', 'boat_back']:
            if midpoint_x > frame_midpoint:  # Bounding box berada di kanan midpoint frame
                yaw = int(1500 + ((midpoint_x - frame_midpoint) / frame_midpoint) * yaw_range)
                direction = "Ke kiri"
            elif midpoint_x < frame_midpoint:  # Bounding box berada di kiri midpoint frame
                yaw = int(1500 - ((frame_midpoint - midpoint_x) / frame_midpoint) * yaw_range)
                direction = "Ke kanan"

        # Membatasi yaw antara min dan max yaw
        yaw = max(self.min_yaw, min(yaw, self.max_yaw))

        rospy.loginfo(f"Midpoint X: {midpoint_x}, Boat Type: {boat_type}, Calculated Yaw: {yaw}, Direction: {direction}")
        return yaw, direction

    def processor(self, frame):
        self.HEIGHT, self.WIDTH = frame.shape[:2]

        # Prediksi objek menggunakan model YOLO
        results = self.model.predict(frame, imgsz=(self.HEIGHT, self.WIDTH), conf=0.5, iou=0.45, stream=True)

        detected_objects = []
        largest_object = None
        largest_area = 0
        current_yaw, yaw_direction = self.initial_yaw, "Diam"

        for r in results:
            if len(r.boxes) > 0:
                for box in r.boxes:
                    cls = int(box.cls)
                    conf = float(box.conf)
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    area = (x2 - x1) * (y2 - y1)

                    if area > largest_area:
                        largest_area = area
                        largest_object = {
                            'class': self.model.names[cls],
                            'confidence': conf,
                            'bbox': (x1, y1, x2, y2)
                        }

                    class_name = self.model.names[cls]
                    detected_objects.append({
                        'class': class_name,
                        'confidence': conf,
                        'bbox': (x1, y1, x2, y2)
                    })
                    
                    rospy.loginfo(f"Detected: {class_name} with confidence {conf:.2f}")

                frame = r.plot()

        # Menggambar garis derajat di frame
        frame = self.draw_degree_lines(frame)

        if largest_object:
            midpoint_x = (largest_object['bbox'][0] + largest_object['bbox'][2]) // 2
            boat_type = largest_object['class']
            current_yaw, yaw_direction = self.calculate_yaw(midpoint_x, boat_type)
            
            info_y = 30
            cv2.putText(frame, f"Boat: {boat_type}", (10, info_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            # Tidak ada objek yang terdeteksi
            info_y = 30
            cv2.putText(frame, "Tidak ada objek terdeteksi", (10, info_y), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
        info_y += 30
        cv2.putText(frame, f"Yaw: {current_yaw}", (10, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        info_y += 30
        cv2.putText(frame, f"Direction: {yaw_direction}", (10, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        info_y += 30
        cv2.putText(frame, f"FPS: {int(self.fps)}", (10, info_y), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        rospy.loginfo(f"Yaw: {current_yaw} {yaw_direction}")

        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.navigation_vision_pub.publish(img_msg)

        return detected_objects

    def calculate_fps(self):
        current_time = rospy.get_time()
        self.fps = 1 / max(current_time - self.prev_time, 1e-6)
        self.prev_time = current_time

    def run(self):
        # Gunakan kamera berdasarkan parameter yang diterima
        cam = cv2.VideoCapture(self.camera_device)
        cam.set(3, self.WIDTH)
        cam.set(4, self.HEIGHT)

        try:
            while not rospy.is_shutdown():
                success, frame = cam.read()
                if not success:
                    rospy.logerr("Camera read failed")
                    break

                self.processor(frame)
                self.calculate_fps()
                self.rate.sleep()

        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")
        
        finally:
            cam.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        Detection().run()
    except rospy.ROSInterruptException:
        pass
