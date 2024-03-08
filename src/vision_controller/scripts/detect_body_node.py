#!/usr/bin/env python3

import rclpy
from vision_controller.nodes.detect_body_im_raw import DetectBodyNode


def main(args=None):
    rclpy.init(args=args)
    node = DetectBodyNode()
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
