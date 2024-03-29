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
from calibration import undistort, camera_param, detect_marker, draw_marker, VectorsToPoseStamped
from rotation import RotationVectorToQuaternion

def pub_data(rvecs, tvecs):
    q = RotationVectorToQuaternion(rvecs)
    tvecs = np.squeeze(tvecs)
    p = PoseStamped()
    p.header.frame_id = "camera"
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = tvecs[0]
    p.pose.position.y = tvecs[1]
    p.pose.position.z = tvecs[2]
    p.pose.orientation.x = q[0]
    p.pose.orientation.y = q[1]
    p.pose.orientation.z = q[2]
    p.pose.orientation.w = q[3]
    pub_pose = rospy.Publisher("/AR/camera_pose", PoseStamped, queue_size=10)
    pub_pose.publish(p)

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    ids, rvecs, tvecs = detect_marker(cv_image, mtx, dist, dictionary, 0.2 )
    if ids is None:
        rospy.loginfo("not found Markers")
    else:
        draw_marker(cv_image, ids, mtx, dist, rvecs, tvecs)
        pub_data(rvecs, tvecs)
        image_message = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        pub.publish(image_message)

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("image_raw", Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'ar_detector'
        bridge = CvBridge()
        pub = rospy.Publisher("/AR/camera_image", Image, queue_size=10)
        fs = cv2.FileStorage(rospy.get_param("calibration_path"), cv2.FILE_STORAGE_READ)
        mtx = fs.getNode("intrinsic").mat()
        dist = fs.getNode("distortion").mat()
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        main()
    except KeyboardInterrupt:
        pass
