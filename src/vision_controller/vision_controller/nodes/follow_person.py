#!/usr/bin/env python3
import json

from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class FollowPersonNode(Node):

    def __init__(self):
        super().__init__("follow_person")

        self.image_raw_subscriber_ = self.create_subscription(
            msg_type=String,
            topic="/detected_colour/position",
            callback=self.position_callback,
            qos_profile=10,
        )

        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Vision Controller has been started")

    def position_callback(self, msg: String):
        self.get_logger().debug("position_callback has been called!")
        position = json.loads(msg.data)
        twist = Twist()
        if position:
            center_x = position["center"][0]
            width = position["width"]

            # center the colour in the image
            if center_x < 83:
                twist.angular.z = 0.5
            elif center_x > 167:
                twist.angular.z = -0.5
            # else:
            # 	twist.angular.z = 0.0

            # if width is smaller than 100, move forward
            if width > 10 and width < 80:
                twist.linear.x = 0.2
            elif width > 100:
                twist.linear.x = -0.1
            # else:
            # 	twist.linear.x = 0.0

            self.get_logger().info(f"Publishing twist: {twist}")
            self.publisher_.publish(twist)
