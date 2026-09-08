#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class PerceptionPipeline(Node):
    def __init__(self):
        super().__init__('perception_node')
        
        # 1. The Ears: Listen to the Gazebo camera
        self.subscription = self.create_subscription(
            Image, '/front_camera/image_raw', self.image_callback, 10)
        
        # 2. The Translator: Converts ROS 2 radio waves into OpenCV pictures
        self.bridge = CvBridge()
        
        # 3. The Brain: Load the smallest, fastest version of YOLO (YOLOv8 Nano)
        self.get_logger().info("Waking up the YOLO brain...")
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info("Camera Bridge & Brain Active: Ready to see!")

    def image_callback(self, msg):
        # Step A: Turn the radio message into a picture
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Step B: Show the picture to the YOLO brain
        # verbose=False stops it from spamming your terminal with text
        results = self.model(cv_image, verbose=False)
        
        # Step C: Ask YOLO to draw neon boxes around what it sees
        annotated_frame = results[0].plot()
        
        # Step D: Show the final image on your screen
        cv2.imshow("Drone Vision: Object Detection", annotated_frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(PerceptionPipeline())
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()