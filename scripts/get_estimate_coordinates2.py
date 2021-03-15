#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import csv
import cv2
import datetime
import numpy as np
import math
from sklearn.cluster import KMeans
import tf
from geometry_msgs.msg import PoseStamped
import math
from ar_func import read_csv, get_weight_position
from ar_func import get_translate_matrix
NODE_NAME = 'get_estimate_coordinates'
input_file_path = rospy.get_param("/get_tmatrix/input_files", "./")
#input_file_path = '/home/rb/ARenv/Pose_2020-07-14_143324'
#output_file_path = '/home/rb/ARenv/Pose_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
pose_file = input_file_path + '/pose.csv'
ar_file = input_file_path + '/ar.csv'

def quaternion_to_vector(quaternion):
    R = tf.transformations.quaternion_matrix(quaternion)[:3,:3]
    V = cv2.Rodrigues(R)[0]
    return V

def RmatTvec_to_cameraMatrix(R,T):
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((T, np.ones(1)))[np.newaxis, :].T))
    return C

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    TQ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def vector_to_matrix(vector):
    R = cv2.Rodrigues(vector)[0]
    return R
def matrix_to_vector(matrix):
    return vector_to_matrix(matrix)

def vector_to_quarernion(vector):
    R = vector_to_matrix(vector)
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((np.zeros(R.shape[1]), np.ones(1)))[np.newaxis, :].T))
    Q = tf.transformations.quaternion_from_matrix(C)
    return Q

def estimate(T_camM, input):
    T = input[:3].astype(np.float32)
    Rvec = quaternion_to_vector(input[3:].astype(np.float32))
    R = vector_to_matrix(Rvec)
    #print(R)
    #print('############ REVERSE #######################')
    #R = np.dot(R, np.array([[1 ,0, 0],[0 ,1, 0],[0 ,0, -1]]))
    #print(R)
    C = RmatTvec_to_cameraMatrix(R, T)
    Est_C = np.dot(T_camM, C)
    Est_pos = Est_C[:3, 3]
    Est_R = Est_C[:3, :3]
    Est_vec = matrix_to_vector(Est_R)
    Est_Quat = vector_to_quarernion(Est_vec)
    return Est_Quat, Est_pos

def callback(msg, args):
    inputTvec, inputQuat, inputTQ = PoseStamped_to_Numpyarray(msg)
    #print('############ REVERSE #######################')
    #print(inputTQ)
    #inputTQ = reverse_xy2(inputTQ)
    #print(inputTQ)
    translate_matrix = args
    Est_Quat, Est_pos = estimate(translate_matrix, inputTQ)
    p = PoseStamped()
    p.header.frame_id = "estimate"
    p.header.stamp = rospy.Time.now()
    p.pose.position.x = Est_pos[0]
    p.pose.position.y = Est_pos[1]
    p.pose.position.z = Est_pos[2]
    p.pose.orientation.x = Est_Quat[0]
    p.pose.orientation.y = Est_Quat[1]
    p.pose.orientation.z = Est_Quat[2]
    p.pose.orientation.w = Est_Quat[3]
    pub = rospy.Publisher("/AR/estimated_pose", PoseStamped, queue_size=10)
    pub.publish(p)

def reverse_xy(pose_quat):
    pose_quat = pose_quat * np.array([[-1, -1, 1, -1, -1, 1, -1],[-1, -1, 1, -1, -1, 1, -1],[-1, -1, 1, -1, -1, 1, -1]])
    return pose_quat
def reverse_xy2(pose_quat):
    pose_quat = pose_quat * np.array([-1, -1, 1, -1, -1, 1, -1])
    return pose_quat
def main():
    try:
        r_poses_quat = read_csv(pose_file).astype(np.float32)
        ar_poses_quat = read_csv(ar_file).astype(np.float32)
        #print('############ REVERSE #######################')


        sort_robot_centers = get_weight_position(r_poses_quat)
        sort_ar_centers = get_weight_position(ar_poses_quat)
        #sort_ar_centers = reverse_xy(sort_ar_centers)
        #print("Centers of Robot poses: ", sort_robot_centers)
        #print("Centers of AR poses: ", sort_ar_centers)
        translate_matrix = get_translate_matrix(sort_robot_centers, sort_ar_centers)
        for (ar_pose, robot_pose) in zip(ar_poses_quat, r_poses_quat):
            Est_Quat, Est_pos = estimate(translate_matrix, ar_pose)
            robot_vec = quaternion_to_vector(robot_pose[3:].astype(np.float32))
            robot_mat = vector_to_matrix(robot_vec)
            #print("estimate[m]: ", Est_pos, ", real[m]: ", robot_pose[:3].astype(np.float32))
            print("diff_pose: ", Est_pos - robot_pose[:3].astype(np.float32))
            print("estimate[quat]: ", Est_Quat, ", real[quat]: ", robot_pose[3:])
            #print("estimate[rad]: ", Est_euler, ", real[rad]: ", robot_euler)
            #print("estimate[deg]: ", [deg * 180 / math.pi for deg in Est_euler], ", real[deg]: ", [deg * 180 / math.pi for deg in robot_euler])
        rospy.init_node(NODE_NAME)
        rospy.Subscriber("AR/camera_pose", PoseStamped, callback, translate_matrix, queue_size=10)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
