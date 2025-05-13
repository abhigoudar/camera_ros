# --------------------------------------------------------------------------

# Author: Abhishek Goudar.

# See LICENSE for the license information

# --------------------------------------------------------------------------

#!/usr/bin/env python3
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.qos
from sensor_msgs.msg import Image


import cv2
import math 

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        # TODO: Parameterize
        # Video source, resolution and model path
        self.img_sub = self.create_subscription(Image, "/camera/image_raw", self.image_cb,
                                                rclpy.qos.qos_profile_system_default)

        self.img_pub = self.create_publisher(Image, 'yolo_detections',
                                             rclpy.qos.qos_profile_system_default)

        self.bridge = CvBridge()
    def image_cb(self, msg):
        self.get_logger().info('Received first image..', once=True)
        image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgba8")
        # image_rgb = cv2.cvtColor(image_raw, cv2.COLOR_RGBA2BGRA)
        self.process_raw_img(image_raw)


    def process_raw_img(self, raw_img):

        image_msg = self.bridge.cv2_to_imgmsg(raw_img, encoding="bgra8")
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera'

        self.img_pub.publish(image_msg)
        self.get_logger().info('Publishing first detections...', once=True)

if __name__ == "__main__":
    rclpy.init()
    try:
        yolo_node = YoloDetectorNode()
        rclpy.spin(yolo_node)
        # yolo_node.vid_cap.release()
        # cv2.destroyAllWindows()
        rclpy.try_shutdown()
    except KeyboardInterrupt:
        pass