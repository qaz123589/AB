#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)
        self.publish= self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret == True:
            self.publish.publish(self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher("topic_img_pub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


