#!/usr/bin/env python3

from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from ament_index_python.packages import get_package_share_directory
import os
import time

import cv2
from cv_bridge import CvBridge

class DetectHandGesturesNode(Node):
    def __init__(self):
        super().__init__("detect_hand_gestures")
        self.get_logger().info("Vision Controller has been started.")

        self.buffer = 0

        self.cv_bridge = CvBridge()

        self.package_share_directory = get_package_share_directory('vision_controller')
        model_path = os.path.join(self.package_share_directory, 'model', 'gesture_recognizer.task')

        # STEP 2: Create an GestureRecognizer object.
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.GestureRecognizerOptions(base_options=base_options)
        self.recognizer = vision.GestureRecognizer.create_from_options(options)

        self.cmd_vel_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.image_raw_subscriber_ = self.create_subscription(
			msg_type=Image,
			topic="/oakd/rgb/preview/image_raw",
			callback=self.image_raw_callback,
			qos_profile=10
		)

    def image_raw_callback(self, image: Image):
        if self.buffer==30 :
            self.get_logger().debug("image_raw_callback has been called!")

            cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")  # Convert to OpenCV format
            img_path = os.path.join(self.package_share_directory, 'tmp_img', 'tmp.jpg')
            cv2.imwrite(img_path, cv_image)  # Save as JPEG

            # STEP 3: Load the input image.
            loaded_image = mp.Image.create_from_file(img_path)

            # STEP 4: Recognize gestures in the input image.
            recognition_result = self.recognizer.recognize(loaded_image)

            try:
                top_gesture = recognition_result.gestures[0][0]
                if top_gesture.category_name == "None":
                    self.get_logger().info("No gesture recognized")
                if top_gesture.category_name == "Thumb_Up":
                    self.spin_start_time = time.time()  # Record start time
                    self.spin_robot()  # Start spinning

                else:
                    self.get_logger().info(f"Top gesture: {top_gesture.category_name}, score: {top_gesture.score}")
            except IndexError:
                self.get_logger().info("No gesture recognized")
            self.buffer = 0
        else:
            self.buffer += 1
            return
        
    def spin_robot(self):
        while time.time() - self.spin_start_time < 10:  # Spin for 10 seconds
            msg = Twist()
            msg.linear.x = 0.5  
            msg.angular.z = 1.0
            self.cmd_vel_pub_.publish(msg)
            time.sleep(0.1)  # Adjust sleep time if needed
