#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import cv2
import datetime
import numpy as np
import math
import tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from rpl.msg import Point2
import os

def callback(robot_pose, camera_pose):
    h = Header()
    h.stamp = rospy.Time.now()
    h.frame_id = 'integrator'
    output = Point2()
    output.header = h
    output.camera = camera_pose.pose
    output.robot = robot_pose.pose.pose
    pub.publish(output)

def main():
    try:
        rospy.init_node(NODE_NAME)

        robot_pose_sub = message_filters.Subscriber("/amcl_pose", PoseWithCovarianceStamped)
        camera_pose_sub = message_filters.Subscriber("/AR/estimated_pose", PoseStamped)

        slop = float(params.thresholds.slop_ms)/1000.0
        ts = message_filters.ApproximateTimeSynchronizer([robot_pose_sub, camera_pose_sub], 10, slop, allow_headerless=True)
        ts.registerCallback(callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'detect_error_position'
        meter_threshold = rospy.get_param("meter_threshold")
        degree_threshold = rospy.get_param("degree_threshold")

        pub = rospy.Publisher("/AR/integrated_pose", Point2, queue_size=10)
        stop_order = GoalID()
        main()
    except KeyboardInterrupt:
        pass