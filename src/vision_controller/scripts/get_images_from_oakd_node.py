#!/usr/bin/env python3

import rclpy
from vision_controller.nodes.get_images_from_oakd import GetImagesFromOakdNode

def main(args=None):
    rclpy.init(args=args)
    node = GetImagesFromOakdNode()
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