#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from ar_func import read_csv
from ar_func import quaternion_to_vector
from ar_func import PoseStamped_to_Numpyarray
from ar_func import estimate

def callback(msg, args):
    inputTvec, inputQuat, inputTQ = PoseStamped_to_Numpyarray(msg)
    translate_matrix = args
    Est_Quat, Est_pos = estimate(translate_matrix, inputTQ)
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = Est_pos[0]
    p.pose.position.y = Est_pos[1]
    p.pose.position.z = Est_pos[2]
    p.pose.orientation.x = Est_Quat[0]
    p.pose.orientation.y = Est_Quat[1]
    p.pose.orientation.z = Est_Quat[2]
    p.pose.orientation.w = Est_Quat[3]
    pub.publish(p)

def main():
    try:
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("/AR/estimated_pose", PoseStamped, callback, reverse_matrix, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    try:
        NODE_NAME = 'demo'
        pose_file = "/home/rb/camera_ws/src/rpl/config/log_image_robot/Pose_2020-12-11_162050/"+ "tmatrix_f_rb.csv"
        tmatrix = read_csv2(pose_file)
        translate_matrix = tmatrix.astype(np.float32)
        print(translate_matrix)
        reverse_matrix = np.linalg.inv(translate_matrix)
        print(reverse_matrix)
        print(np.dot(translate_matrix, reverse_matrix))
        pub = rospy.Publisher("/AR/demo_pose", PoseStamped, queue_size=10)
        main()

    except KeyboardInterrupt:
        pass