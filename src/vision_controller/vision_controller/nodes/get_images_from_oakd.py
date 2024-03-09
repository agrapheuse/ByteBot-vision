#!/usr/bin/env python3
import os

import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class GetImagesFromOakdNode(Node):

    def __init__(self):
        super().__init__("get_images_from_oakd")
        self.image_raw_subscriber_ = self.create_subscription(
            msg_type=Image,
            topic="/oakd/rgb/preview/image_raw",
            callback=self.image_raw_callback,
            qos_profile=10,
        )
        self.bridge = CvBridge()
        # Directory where images will be saved
        self.image_save_directory = "/home/parallels/images"
        if not os.path.exists(self.image_save_directory):
            os.makedirs(self.image_save_directory)
        self.get_logger().info("Vision Controller has been started.")

    def image_raw_callback(self, image: Image):
        try:
            self.get_logger().info("Starting image_raw_callback")
            # Convert the ROS2 Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
            self.get_logger().info("Converted image")

            # Optionally save the image for later use or analysis
            image_filename = os.path.join(
                self.image_save_directory,
                f"frame_{self.get_clock().now().nanoseconds}.jpg",
            )
            cv2.imwrite(image_filename, cv_image)
            self.get_logger().info("Wrote image")
        except Exception as e:
            self.get_logger().error("Failed to process image: " + str(e))
