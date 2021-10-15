#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class view_images(object):
    def __init__(self, subtopic, width, height):
        self.subscriber1 = rospy.Subscriber(subtopic[0], Image, self.callback1)
        self.subscriber2 = rospy.Subscriber(subtopic[1], Image, self.callback2)
        self.subscriber3 = rospy.Subscriber(subtopic[2], Image, self.callback3)

        self.bridge = CvBridge()
        self.brank = np.zeros((height, width, 3),np.uint8)
        self.image1 = np.zeros((height, width, 3),np.uint8)
        self.image2 = np.zeros((height, width, 3),np.uint8)
        self.image3 = np.zeros((height, width, 3),np.uint8)
        self.mergeImg = None

    def merge(self):
        mergeImg1 = np.hstack((self.image1, self.image2))
        mergeImg2 = np.hstack((self.image3, self.brank))
        self.mergeImg = np.vstack((mergeImg1, mergeImg2))

    def show(self):
        cv2.imshow(w_name, self.mergeImg)
        cv2.waitKey(1)

    def callback1(self, message):
        self.image1 = cv2.resize(self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'), dsize=(height, width))
        self.merge()
        self.show()

    def callback2(self, message):
        self.image2 = cv2.resize(self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'), dsize=(height, width))
        self.merge()
        self.show()

    def callback3(self, message):
        self.image3 = cv2.resize(self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'), dsize=(height, width))
        self.merge()
        self.show()

def main():
    try:
        pub = view_images(topic_name, width, height)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'view_images'
        rospy.init_node(NODE_NAME)
        width = rospy.get_param("width", 480)
        height = rospy.get_param("height", 480)
        w_name = rospy.get_param("~w_name")
        topic_name = [
            "image1",
            "image2",
            "image3",
#            "/" + "C1" + "/image_raw",
#            "/" + "C2" + "/image_raw",
#            "/" + "C3" + "/image_raw",
        ]
        main()
    except KeyboardInterrupt:
        pass
