#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys
sys.path.append('../../../../scripts/')
from calibration import undistort, camera_param

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    dst, _ = undistort(cv_image, DIM, K, D)
    image_message = bridge.cv2_to_imgmsg(dst, encoding="bgr8")
    pub.publish(image_message)

def main():
    try:
        rospy.init_node(NODE_NAME)
        topic_name = "/" + name + "/image_raw"
        rospy.Subscriber(topic_name, Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'calibration'
        bridge = CvBridge()
        name = rospy.get_param("name", "camera")
        topic_name = "/" + name + "/calib_image"
        pub = rospy.Publisher(topic_name, Image, queue_size=10)

        DIM, K, D = camera_param('../../../../config/calibration.yml')

        main()
    except KeyboardInterrupt:
        pass
