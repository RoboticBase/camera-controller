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

from fisheye_calibrate import camera_param
from RosMsgs import VectorsToPoseStamped

def draw_marker(frame, ids, mtx, dist, rvecs, tvecs):
    for i in range(ids.size):
        r = np.squeeze(rvecs[i])
        t = np.squeeze(tvecs[i])
        aruco.drawAxis(frame, mtx, dist, r, t, 0.1)
        R = cv2.Rodrigues(r)[0]
        T = t[np.newaxis, :].T
        proj_matrix = np.hstack((R, T))
        euler_angle = cv2.decomposeProjectionMatrix(proj_matrix)[6] # [deg]
        cv2.putText(frame, "X: %.1f cm" % (t[0] * 100),  (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
        cv2.putText(frame, "Y: %.1f cm" % (t[1] * 100),  (0, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
        cv2.putText(frame, "Z: %.1f cm" % (t[2] * 100),  (0, 90), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
        cv2.putText(frame, "R: %.1f deg" % (euler_angle[0]),  (0, 130), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
        cv2.putText(frame, "P: %.1f deg" % (euler_angle[1]),  (0, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
        cv2.putText(frame, "Y: %.1f deg" % (euler_angle[2]),  (0, 180), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255))
    return frame

def detect_marker(frame, dictionary, K, D, marker_size=0.184):
    parameters =  aruco.DetectorParameters_create()
    parameters.cornerRefinementMethod = aruco.CORNER_REFINE_CONTOUR
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, dictionary, parameters=parameters)#, cameraMatrix=K, distCoeff=D)
    rvecs, tvecs, _objPoints = aruco.estimatePoseSingleMarkers(corners, marker_size, K, D)
    return corners, ids, rvecs, tvecs

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
        rospy.Subscriber(topic_name + "/calib_image", Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'ar_detector'
        bridge = CvBridge()
        rospy.init_node(NODE_NAME)
        name = rospy.get_param("~name", "camera")
        topic_name = "/" + name

        pub = rospy.Publisher(topic_name + "/AR/camera_image", Image, queue_size=10)
        pub_pose = rospy.Publisher(topic_name + "/AR/camera_pose", PoseStamped, queue_size=10)

        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        calib_path = rospy.get_param("~calibparam")
        DIM, K, D = camera_param(calib_path)

        main()
    except KeyboardInterrupt:
        pass
