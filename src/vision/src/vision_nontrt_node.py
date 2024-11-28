#!/usr/bin/env python3

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from pathlib import Path
import sys
import os

from ultralytics import YOLO

class Detection:
    def __init__(self):
        rospy.init_node('vision_node', anonymous=True)
        self.rate = rospy.Rate(20)

        # Publisher
        self.navigation_vision_pub = rospy.Publisher('/robot/vision/navigation', Image, queue_size=1)

        # Load PyTorch Model (.pt)
        ROOT = Path(__file__).resolve().parents[0]
        self.model = YOLO(model=f'{ROOT}/model/best.pt', task='detect')

        # Constants
        self.WIDTH = 640
        self.HEIGHT = 480
        
        # Variables
        self.bridge = CvBridge()
        self.fps = 0
        self.prev_time = rospy.get_time()

    def processor(self, frame):
        # Predict with slightly adjusted parameters for PyTorch model
        results = self.model.predict(frame, imgsz=(self.WIDTH, self.HEIGHT), conf=0.5, iou=0.45, stream=True)

        for r in results:
            if len(r.boxes) > 0:
                for box in r.boxes:
                    # Extract class and confidence
                    cls = int(box.cls)  # Class ID
                    conf = float(box.conf)  # Confidence score

                    # Get class name
                    class_name = self.model.names[cls]
                    rospy.loginfo(f"Detected: {class_name} with confidence {conf:.2f}")

                # Plot bounding boxes on the frame
                frame = r.plot()

        # Publish processed frame
        img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.navigation_vision_pub.publish(img_msg)

    def calculate_fps(self):
        current_time = rospy.get_time()
        self.fps = 1 / max(current_time - self.prev_time, 1e-6)
        self.prev_time = current_time
        rospy.loginfo(f"FPS: {int(self.fps)}")

    def run(self):
        cam = cv2.VideoCapture(2)
        cam.set(3, self.WIDTH)
        cam.set(4, self.HEIGHT)

        try:
            while not rospy.is_shutdown():
                success, frame = cam.read()
                if not success:
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