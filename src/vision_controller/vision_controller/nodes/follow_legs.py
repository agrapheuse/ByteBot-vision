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
        self.no_legs_detected_timer = self.create_timer(5.0, self.no_legs_detected_callback)

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
                self.get_logger().info(f"Detected {len(legs)} leg(s)")
                self.reset_no_legs_detected_timer()
                x, y, w, h = legs[0]  # assuming the first detected part is the target
                center_x, center_y = x + w // 2, y + h // 2
                twist = Twist()
                # update twist.linear.x, twist.angular.z based on center_x to move the Turtlebot
                if center_x < 100:
                    twist.angular.z = 0.5
                elif center_x > 180:
                    twist.angular.z = -0.5
                twist.linear.x = 0.2
                self.publisher_.publish(twist)
                self.get_logger().debug("Published a twist message")
        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert ROS Image to OpenCV: {e}")
