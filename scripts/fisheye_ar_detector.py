#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2.aruco as aruco
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix

import sys
sys.path.append('../../../../scripts/')
from calibration import undistort, camera_param, detect_marker, draw_marker
from RosMsgs import VectorsToPoseStamped

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    corners, ids, rvecs, tvecs = detect_marker(gray, dictionary, K, np.zeros((4, 1)))
    if ids is None:
        rospy.loginfo("not found Markers")
    else:
        draw_marker(cv_image, ids, K, np.zeros((4, 1)), rvecs, tvecs)
        p = VectorsToPoseStamped(rvecs, tvecs, "camera")
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")

        pub_pose.publish(p)
        pub.publish(image_message)

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber(topic_name + "/calib_image", Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'ar_detector'
        bridge = CvBridge()
        name = rospy.get_param("name", "camera")
        topic_name = "/" + name

        pub = rospy.Publisher(topic_name + "/AR/camera_image", Image, queue_size=10)
        pub_pose = rospy.Publisher(topic_name + "/AR/camera_pose", PoseStamped, queue_size=10)

        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        DIM, K, D = camera_param('../../../../config/calibration.yml')

        main()
    except KeyboardInterrupt:
        pass
