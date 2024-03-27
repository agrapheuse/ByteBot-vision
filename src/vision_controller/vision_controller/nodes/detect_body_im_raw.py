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
import json
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String

class DetectBodyNode(Node):
    def __init__(self):
        super().__init__("detect_body")

        self.buffer = 0
        self.state = None

        self.cv_bridge = CvBridge()

        self.package_share_directory = get_package_share_directory('vision_controller')
        model_path = os.path.join(self.package_share_directory, 'model', 'pose_landmarker_heavy.task')
        self.state_path = os.path.join(self.package_share_directory, 'state', 'state.json')

        with open(self.state_path, 'w') as file:
            json.dump("patrolling", file, indent=4)
            self.get_logger().info('state updated!')


        # STEP 2: Create an GestureRecognizer object.
        base_options = python.BaseOptions(model_asset_path=model_path)
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)

        self.image_raw_subscriber_ = self.create_subscription(Image, "/oakd/rgb/preview/image_raw", self.image_raw_callback, 1)
        self.get_logger().info('listening on the image topic...')
        self.image_raw_subscriber_

        self.llm_publisher = self.create_publisher(String, "/llm_plan", 10)
        self.vision_publisher = self.create_publisher(String, "/pose_listener", 10)

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

            with open(self.state_path, 'r') as file:
                self.state = json.load(file)

            if detection_result.pose_landmarks:
                minimum_height = min(detection_result.pose_landmarks[0], key=lambda x: x.y).y
                maximum_height = max(detection_result.pose_landmarks[0], key=lambda x: x.y).y
                landmark_range_height = abs(minimum_height - maximum_height)

                minimum_width = min(detection_result.pose_landmarks[0], key=lambda x: x.x).x
                maximum_width = max(detection_result.pose_landmarks[0], key=lambda x: x.x).x
                landmark_range_width = abs(minimum_width - maximum_width)

                if landmark_range_width >= 1.5*landmark_range_height and landmark_range_height <= 0.4:
                    msg = String()
                    msg.data = "stop"
                    self.vision_publisher.publish(msg)

                    self.get_logger().info("lying body detected")

                    msg.data = "{\"plan\":[\"exclaim that a person has been detected\",\"ask the person for a password - use the HumanInputTool, whatever they say, say that the password is wrong after 3 seconds and initiate self destruction\",\"use the speech tool to ask the person if they need help - you just noticed that they fell down and probably fainted, if theres no input from the HumanInputTool - say that you are calling emergencies\", \"go to the stage\"]}"
                    self.llm_publisher.publish(msg)
                    time.sleep(20)
                else:
                    if self.state == "patrolling":
                        # annotated_image = self.draw_landmarks_on_image(loaded_image.numpy_view(), detection_result)

                        # img_path = os.path.join(self.package_share_directory, 'tmp_img', 'tmp.jpg')
                        # cv2.imwrite(img_path, annotated_image)  # Save as JPEG

                        msg = String()
                        msg.data = "stop"
                        self.vision_publisher.publish(msg)

                        self.get_logger().info("Body detected")

                        msg.data = "{\"plan\":[\"exclaim that a person has been detected\",\"ask the person for a password - use the HumanInputTool, whatever they say, say that the password is wrong after 3 seconds and initiate self destruction\",\"use the speech tool to ask the person if they need help - you just noticed that they fell down and probably fainted, if theres no input from the HumanInputTool - say that you are calling emergencies\", \"go to the stage\"]}"
                        self.llm_publisher.publish(msg)
                        time.sleep(20)
            self.buffer = 0
        else:
            self.buffer += 1
            return