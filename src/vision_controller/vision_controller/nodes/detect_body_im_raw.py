#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.framework.formats import landmark_pb2
from mediapipe import solutions
from ament_index_python.packages import get_package_share_directory
import numpy as np

import os
from sensor_msgs.msg import Image

class DetectBodyNode(Node):
    def __init__(self):
        super().__init__("detect_body")

        self.buffer = 0
        self.id=0

        self.cv_bridge = CvBridge()

        self.package_share_directory = get_package_share_directory('vision_controller')
        model_path = os.path.join(self.package_share_directory, 'model', 'pose_landmarker_heavy.task')

        # STEP 2: Create an GestureRecognizer object.
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)

        self.image_raw_subscriber_ = self.create_subscription(
			msg_type=Image,
			topic="/oakd/rgb/preview/image_raw",
			callback=self.image_raw_callback,
			qos_profile=10
        )

    def draw_landmarks_on_image(self, rgb_image, detection_result):
        pose_landmarks_list = detection_result.pose_landmarks
        annotated_image = np.copy(rgb_image)

        # Loop through the detected poses to visualize.
        for idx in range(len(pose_landmarks_list)):
            pose_landmarks = pose_landmarks_list[idx]

            # Draw the pose landmarks.
            pose_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            pose_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in pose_landmarks
            ])
            solutions.drawing_utils.draw_landmarks(
                annotated_image,
                pose_landmarks_proto,
                solutions.pose.POSE_CONNECTIONS,
                solutions.drawing_styles.get_default_pose_landmarks_style())
        return annotated_image


    def image_raw_callback(self, image: Image):
        if self.buffer==30 :
            self.get_logger().debug("image_raw_callback has been called!")

            cv_image = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")  # Convert to OpenCV format
            img_path = os.path.join(self.package_share_directory, 'tmp_img', 'tmp.jpg')
            cv2.imwrite(img_path, cv_image)  # Save as JPEG

            # STEP 3: Load the input image.
            loaded_image = mp.Image.create_from_file(img_path)

            # STEP 4: Detect pose landmarks from the input image.
            detection_result = self.detector.detect(loaded_image)

            if not detection_result.pose_landmarks:
                self.get_logger().info("No body detected")
            else:
                self.get_logger().info("Body detected")

                annotated_image = self.draw_landmarks_on_image(loaded_image.numpy_view(), detection_result)

                # img_path = os.path.join(self.package_share_directory, 'tmp_img', 'tmp.jpg')
                # cv2.imwrite(img_path, annotated_image)  # Save as JPEG

                minimum = min(detection_result.pose_landmarks[0], key=lambda x: x.y).y
                maximum = max(detection_result.pose_landmarks[0], key=lambda x: x.y).y
                landmark_range = abs(minimum - maximum)

                if landmark_range <= 0.4:
                    self.get_logger().info("lying body detected")

            self.buffer = 0
        else:
            self.buffer += 1
            return