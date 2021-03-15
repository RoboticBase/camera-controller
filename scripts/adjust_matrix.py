#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
import csv
import datetime
import numpy as np
import tf
import os
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseStamped
from ar_func import read_csv, latest_file

def main():
    try:
        pass
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'adjust_matrix'
        tmatrix_path = rospy.get_param("matrix_path")
        record_path = rospy.get_param("record_path")
        adjust_path = rospy.get_param("adjust_path")
        latest_time = latest_file(tmatrix_path, "TranslateM_", ".csv")
        tmatrix_file = tmatrix_path + latest_time
        print(tmatrix_file)
        latest_time = latest_file(record_path, "Point2_Diff_", ".csv")
        record_file = record_path + latest_time
        print(record_file)

    except KeyboardInterrupt:
        pass