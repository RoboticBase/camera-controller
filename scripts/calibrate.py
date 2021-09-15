#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Calibrator(object):
    def __init__(self, pubtopic, subtopic, intrinsic, distortion):
        self.publisher = rospy.Publisher(pubtopic, Image, queue_size=10)
        self.subscriber = rospy.Subscriber(subtopic, Image, self.callback, queue_size=10)
        self.message = Image()
        self.bridge = CvBridge()
        self.intrinsic = intrinsic
        self.distortion = distortion

    def callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        dst = cv2.undistort(cv_image, self.intrinsic, self.distortion)
        self.message = self.bridge.cv2_to_imgmsg(dst, encoding="bgr8")
        self.publish()

    def publish(self):
        self.publisher.publish(self.message)

def main():
    try:
        node = Calibrator(pubtopic, subtopic, intrinsic, distortion)
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'calibration'
        pubtopic = "calib_image"
        fs = cv2.FileStorage(rospy.get_param("calibration_path"), cv2.FILE_STORAGE_READ)
        intrinsic = fs.getNode("intrinsic").mat()
        distortion = fs.getNode("distortion").mat()
        name = rospy.get_param("name", "camera")
        subtopic = "/" + name + "/image_raw"

        rospy.init_node(NODE_NAME)
        main()
    except KeyboardInterrupt:
        pass
