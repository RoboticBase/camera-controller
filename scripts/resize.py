#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def callback(msg):
    size = (400, 400)
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    dst = cv2.resize(cv_image, size)
    image_message = bridge.cv2_to_imgmsg(dst, encoding="bgr8")
    pub.publish(image_message)

def main():
    try:
        rospy.Subscriber(topic, Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'resize'
        rospy.init_node(NODE_NAME)
        bridge = CvBridge()
        topic = rospy.get_param("~topic", "/image_raw")
        pubtopic = topic + "/resized"
        pub = rospy.Publisher(pubtopic, Image, queue_size=10)
        main()
    except KeyboardInterrupt:
        pass
