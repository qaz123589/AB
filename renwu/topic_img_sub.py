#!/usr/bin/python3
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class ImageSubscriber(Node):

    X = 0
    Y = 0
    def __init__(self, name):
        super().__init__(name)
        self.sub = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.publish = self.create_publisher(Image,'image',10)

    def object_detect(self, image):
        lower_red = np.array([0, 165, 46])
        upper_red = np.array([10, 200, 255])
        gs_image = cv2.GaussianBlur(image, (3, 3), 0)
        hsv = cv2.cvtColor(gs_image, cv2.COLOR_BGR2HSV)
        kernel = np.ones([3, 3], dtype=np.uint8)
        erode = cv2.erode(hsv, kernel, iterations=2)
        Range = cv2.inRange(erode, lower_red, upper_red)

        contours, hierarchy = cv2.findContours(Range, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        for cnt in contours:
            if cnt.shape[0] < 90:
                continue

            (x, y, w, h) = cv2.boundingRect(cnt)
            image = cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            self.X = float(x + w / 2)
            self.Y = float(y + h / 2)
            print(self.X,self.Y)
        cv2.imshow("object", image)
        cv2.waitKey(10)

    def listener_callback(self, data):

        image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.object_detect(image)
        try:
            self.publish.publish(self.cv_bridge.cv2_to_imgmsg(image,'bgr8'))
        except CvBridgeError as e:
            print (e)    
     
 
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber("topic_img_sub")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()                                  
