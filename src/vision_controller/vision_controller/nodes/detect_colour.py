#!/usr/bin/env python3
import json

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class DetectColourNode(Node):

    def __init__(self):
        super().__init__("detect_colour")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("hue1_lower", [0, 120, 70]),  # lower range for the first colour
                ("hue1_upper", [10, 255, 255]),  # upper range for the first colour
                (
                    "hue2_lower",
                    [170, 120, 70],
                ),  # lower range for the second colour (if needed)
                (
                    "hue2_upper",
                    [180, 255, 255],
                ),  # upper range for the second colour (if needed)
            ],
        )

        self.image_raw_subscriber_ = self.create_subscription(
            msg_type=Image,
            topic="/oakd/rgb/preview/image_raw",
            callback=self.image_raw_callback,
            qos_profile=10,
        )
        self.publisher_ = self.create_publisher(String, "/detected_colour/position", 10)
        self.bridge = CvBridge()
        self.get_logger().info("Vision Controller has been started")

    def image_raw_callback(self, image: Image):
        self.get_logger().debug("image_raw_callback has been called!")
        cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        position = self.detect_colour(cv_image)
        if position:
            self.get_logger().info(f"Detected colour at position: {position}")
            self.publisher_.publish(String(data=json.dumps(position)))

    def detect_colour(self, cv_image):
        # get the colour ranges from the parameters
        hue1_lower = np.array(
            self.get_parameter("hue1_lower").get_parameter_value().integer_array_value,
            dtype=np.uint8,
        )
        hue1_upper = np.array(
            self.get_parameter("hue1_upper").get_parameter_value().integer_array_value,
            dtype=np.uint8,
        )

        # convert the image to the HSV colour space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # create a mask
        mask = cv2.inRange(hsv_image, hue1_lower, hue1_upper)

        try:
            hue2_lower_param = self.get_parameter("hue2_lower").value
            hue2_upper_param = self.get_parameter("hue2_upper").value
        except ParameterUninitializedException:
            hue2_lower_param = False
            hue2_upper_param = False

        if hue2_lower_param and hue2_upper_param:
            hue2_lower = np.array(
                self.get_parameter("hue2_lower")
                .get_parameter_value()
                .integer_array_value,
                dtype=np.uint8,
            )
            hue2_upper = np.array(
                self.get_parameter("hue2_upper")
                .get_parameter_value()
                .integer_array_value,
                dtype=np.uint8,
            )
            mask2 = cv2.inRange(hsv_image, hue2_lower, hue2_upper)
            mask = cv2.bitwise_or(mask, mask2)  # combine the masks

        # find the contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            center = (x + w // 2, y + h // 2)
            return {"center": center, "width": w, "height": h}
        return None
