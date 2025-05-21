#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2

class DecompressNode(Node):
    def __init__(self):
        super().__init__('decompress_node')
        self.sub = self.create_subscription(
            CompressedImage,
            '/kinect/image_raw/compressed',
            self.callback,
            10)
        self.pub = self.create_publisher(Image, '/kinect/image_uncompressed', 10)
        self.bridge = CvBridge()

    def callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        uncompressed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        
        # Copy the header from the input message to maintain frame_id and timestamp
        uncompressed_msg.header = msg.header
        
        # If the header is empty, set a default frame_id
        if not uncompressed_msg.header.frame_id:
            uncompressed_msg.header.frame_id = 'kinect_frame'  # or whatever your camera frame is called
            
        self.pub.publish(uncompressed_msg)

def main():
    rclpy.init()
    node = DecompressNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
