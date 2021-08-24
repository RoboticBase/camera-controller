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
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/C1/AR/camera_image", Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'resize'
        bridge = CvBridge()
        name = rospy.get_param("name", "camera")
        topic_name = "/" + name + "/resized"
        pub = rospy.Publisher(topic_name, Image, queue_size=10)
        main()
    except KeyboardInterrupt:
        pass
