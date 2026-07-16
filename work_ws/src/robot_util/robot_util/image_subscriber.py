#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriberNode(Node):
    """图像订阅节点，订阅深度和RGB图像话题但不做任何处理"""
    
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 订阅深度图像话题
        self.depth_subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/depth0/image_raw',
            self.depth_callback,
            10
        )
        
        # 订阅RGB图像话题
        self.rgb_subscription = self.create_subscription(
            Image,
            '/ascamera_hp60c/camera_publisher/rgb0/image',
            self.rgb_callback,
            10
        )
        
        self.get_logger().info('Image subscriber node started')
    
    def depth_callback(self, msg):
        """深度图像回调函数，不做任何操作直接返回"""
        pass
    
    def rgb_callback(self, msg):
        """RGB图像回调函数，不做任何操作直接返回"""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
