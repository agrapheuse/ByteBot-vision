#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
import json
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String



class ChangeStateNode(Node):
    def __init__(self):
        super().__init__("change_state")

        self.state = None

        self.package_share_directory = get_package_share_directory('vision_controller')
        self.state_path = os.path.join(self.package_share_directory, 'state', 'state.json')

        self.nav_module_listener = self.create_subscription(String, "/nav_vision_topic", self.nav_callback, 10)
        self.nav_module_listener
        self.get_logger().info("listening on /nav_vision_topic...")

        with open(self.state_path, 'r') as file:
            self.state = json.load(file)

    def nav_callback(self, msg):
        self.get_logger().info('nav_callback has been called!')
        if msg.data == 'patrolling':
            self.state = 'patrolling'
            self.save_state()
        if msg.data == 'None':
            self.state = 'None'
            self.save_state()

    def save_state(self):
        with open(self.state_path, 'w') as file:
            json.dump(self.state, file, indent=4)
            self.get_logger().info('state updated!')


