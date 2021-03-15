#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import cv2
import datetime
import numpy as np
import tf
import os
from sklearn.cluster import KMeans
from geometry_msgs.msg import PoseStamped
from ar_func import read_csv, get_weight_position, get_translate_matrix, quaternion_to_vector, vector_to_matrix, matrix_to_vector
from ar_func import estimate

def PoseStamped_to_Numpyarray(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    TQ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

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


def latest_dir(path, head_str):
    latest_time = datetime.datetime.min
    files = os.listdir(path)
    files_dir = [f for f in files if os.path.isdir(os.path.join(path, f))]
    for dir in files_dir:
        if not dir.startswith(head_str):
            continue
        dir_time = datetime.datetime.strptime(dir.lstrip(head_str), '%Y-%m-%d_%H%M%S')
        if dir_time > latest_time:
            latest_time = dir_time
            latest_dir = dir
    return latest_dir

def save_csv(matrix):
    buf = str(matrix[0][0]) + " ,"
    buf = buf + str(matrix[0][1]) + " ,"
    buf = buf + str(matrix[0][2]) + " ,"
    buf = buf + str(matrix[0][3]) + "\n"
    buf = buf + str(matrix[1][0]) + " ,"
    buf = buf + str(matrix[1][1]) + " ,"
    buf = buf + str(matrix[1][2]) + " ,"
    buf = buf + str(matrix[1][3]) + "\n"
    buf = buf + str(matrix[2][0]) + " ,"
    buf = buf + str(matrix[2][1]) + " ,"
    buf = buf + str(matrix[2][2]) + " ,"
    buf = buf + str(matrix[2][3]) + "\n"
    buf = buf + str(matrix[3][0]) + " ,"
    buf = buf + str(matrix[3][1]) + " ,"
    buf = buf + str(matrix[3][2]) + " ,"
    buf = buf + str(matrix[3][3]) + "\n"
    print(buf)
    with open(output_path, mode='a') as f:
        f.write(buf)
if __name__ == '__main__':
    try:
        NODE_NAME = 'create_matrix'
        file_path = rospy.get_param("file_path")
        latest_time = latest_dir(file_path, "Pose_")
        matrix_path = rospy.get_param("matrix_path")
        output_path = matrix_path + 'TranslateM_' + datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S') + '.csv'
        print("READED position from ", latest_time)
        pose_file = file_path + latest_time + '/pose.csv'
        ar_file = file_path + latest_time + '/ar.csv'
        r_poses_quat = read_csv(pose_file).astype(np.float32)
        ar_poses_quat = read_csv(ar_file).astype(np.float32)
        sort_robot_centers = get_weight_position(r_poses_quat)
        sort_ar_centers = get_weight_position(ar_poses_quat)
        #print('############ REVERSE #######################')
        #sort_ar_centers = reverse_xy(sort_ar_centers)
        print("Centers of Robot poses: ", sort_robot_centers)
        print("Centers of AR poses: ", sort_ar_centers)
        translate_matrix = get_translate_matrix(sort_robot_centers, sort_ar_centers)
        save_csv(translate_matrix)
        for (ar_pose, robot_pose) in zip(ar_poses_quat, r_poses_quat):
            Est_Quat, Est_pos = estimate(translate_matrix, ar_pose)
            robot_vec = quaternion_to_vector(robot_pose[3:].astype(np.float32))
            robot_mat = vector_to_matrix(robot_vec)
            #print("estimate[m]: ", Est_pos, ", real[m]: ", robot_pose[:3].astype(np.float32))
            print("diff_pose: ", Est_pos - robot_pose[:3].astype(np.float32))
            print("estimate[quat]: ", Est_Quat, ", real[quat]: ", robot_pose[3:])
            #print("estimate[rad]: ", Est_euler, ", real[rad]: ", robot_euler)
            #print("estimate[deg]: ", [deg * 180 / math.pi for deg in Est_euler], ", real[deg]: ", [deg * 180 / math.pi for deg in robot_euler])
    except KeyboardInterrupt:
        pass
