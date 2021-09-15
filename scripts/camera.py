#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(object):
    def __init__(self, device_id, topicname, width, height):
        self.publisher = rospy.Publisher(topicname, Image, queue_size=10)
        self.message = Image()
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(device_id)
        self.cap.set(cv2.CAP_PROP_FPS, 10)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.rate = rospy.Rate(10)

        self.ret, self.im = self.read()

    def read(self):
        return self.cap.read()

    def publish(self):
        self.ret, self.im = self.read()
        image_message = self.bridge.cv2_to_imgmsg(self.im, encoding="bgr8")
        self.publisher.publish(image_message)

    def repeat(self):
        if self.ret == False:
            return
        self.publish()
        self.rate.sleep()

    def finalize(self):
        self.cap.release()

def main():
    try:
        pub = CameraPublisher(device_id, topic_name, width, height)
        while not rospy.is_shutdown():
            pub.repeat()
        pub.finalize()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'camera'
        device_id = rospy.get_param("device_id")
        width = rospy.get_param("width", 1920)
        height = rospy.get_param("height", 1080)
        name = rospy.get_param("name", "camera")
        rospy.init_node(NODE_NAME)
        topic_name = "/" + name + "/image_raw"
        main()
    except KeyboardInterrupt:
        pass
