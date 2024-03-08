#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image


class FollowLegsNode(Node):
    def __init__(self):
        super().__init__("follow_legs")
        self.image_raw_subscriber_ = self.create_subscription(
            msg_type=Image,
            topic="/oakd/rgb/preview/image_raw",
            callback=self.image_callback,
            qos_profile=10,
        )
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cv_bridge = CvBridge()
        haar_cascade_path = "/home/parallels/vision_ws/haarcascade_lowerbody.xml"
        self.lower_body_cascade = cv2.CascadeClassifier(haar_cascade_path)
        # if the file is not found, raise an error
        if self.lower_body_cascade.empty():
            raise FileNotFoundError(
                f"Could not load the cascade classifier xml file at {haar_cascade_path}"
            )
        self.get_logger().info("FollowLegsNode has been created")
        # if no legs have been detected for a few seconds, send a log message
        self.no_legs_detected_timer = self.create_timer(
            5.0, self.no_legs_detected_callback
        )

    def no_legs_detected_callback(self):
        self.get_logger().info("No legs detected for 5 seconds")

    def reset_no_legs_detected_timer(self):
        self.no_legs_detected_timer.cancel()
        self.no_legs_detected_timer = self.create_timer(
            5.0, self.no_legs_detected_callback
        )

    def image_callback(self, msg):
        try:
            self.get_logger().debug("Received an image")
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            legs = self.lower_body_cascade.detectMultiScale(
                gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30)
            )

            if len(legs) > 0:
                self.get_logger().info(f"Detected {len(legs)} legs")
                # Reset the timer each time legs are detected
                self.reset_no_legs_detected_timer()
                x, y, w, h = legs[0]  # Assuming the first detected part is the target
                center_x = x + w // 2

                # Image frame dimensions (assuming these or get from camera info)
                frame_width = cv_image.shape[1]
                frame_height = cv_image.shape[0]

                # Calculate the error between the center of the detected legs and the center of the image
                error_x = center_x - frame_width / 2

                # Control parameters
                linear_speed = 0.5  # Constant linear speed
                angular_speed_factor = 0.005  # Adjust based on your needs

                twist = Twist()
                twist.linear.x = float(
                    linear_speed * (1 - abs(error_x) / (frame_width / 2))
                )  # Slow down as the error increases
                twist.angular.z = float(
                    -error_x * angular_speed_factor
                )  # Turn rate proportional to error_x

                # Safety measure to stop if too close (adjust 100 based on your scenario)
                if w * h > (frame_width * frame_height / 4):
                    twist.linear.x = (
                        0.0  # Stop if the detected legs are too large/close
                    )

                self.publisher_.publish(twist)
                self.get_logger().debug("Published a twist message")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert ROS Image to OpenCV: {e}")
