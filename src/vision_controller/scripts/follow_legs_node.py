#!/usr/bin/env python3

import rclpy
from vision_controller.nodes.follow_legs import FollowLegsNode


def main(args=None):
    rclpy.init(args=args)
    node = FollowLegsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Destroy the node explicitly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
