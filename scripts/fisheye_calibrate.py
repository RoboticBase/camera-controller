#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
import cv2
import glob
import yaml
import numpy as np

def read_yml(path):
    with open(path, 'r') as yml:
        f = yaml.safe_load(yml)
    return f

def camera_param(path):
    f = read_yml(path)
    DIM = np.array(f['DIM'])
    K = np.array(f['K'])
    D = np.array(f['D'])
    return DIM, K, D

def undistort(img, DIM, K, D):
    dim2=None
    dim3=None
    balance=1
    dim1 = img.shape[:2][::-1]
    assert dim1[0]/dim1[1] == DIM[0]/DIM[1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    scaled_K = K * dim1[0] / DIM[0]
    scaled_K[2][2] = 1.0
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)

    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img, new_K

def old_undistort(img, DIM, K, D):
    h1, w1 = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return undistorted_img, map1, map2

def Capture(cap, dirname, check="", pattern_size=.0, i=0):
    ret, frame = cap.read()
    while ret == True:
        cv2.imshow('Capture', frame)
        key = cv2.waitKey(50)
        if key == 27: # ESC
            break
        elif key == ord('s'):
            print("capture")
            if check == "Chessboard":
                if not check_board(frame, pattern_size):
                    continue
            if not os.path.exists(dirname):
                os.mkdir(dirname)
            cv2.imwrite(dirname + "/%03.f"%(i)+".png", frame)
            i+=1
        ret, frame = cap.read()

def callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    dst, _ = undistort(cv_image, DIM, K, D)
    image_message = bridge.cv2_to_imgmsg(dst, encoding="bgr8")
    pub.publish(image_message)

def main():
    try:
        topic_name = "/" + name + "/image_raw"
        rospy.Subscriber(topic_name, Image, callback, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'calibration'
        rospy.init_node(NODE_NAME)
        bridge = CvBridge()
        name = rospy.get_param("~name", "camera")
        topic_name = "/" + name + "/calib_image"
        pub = rospy.Publisher(topic_name, Image, queue_size=10)
        calib_path = rospy.get_param("~calibparam")
        DIM, K, D = camera_param(calib_path)

        main()
    except KeyboardInterrupt:
        pass
