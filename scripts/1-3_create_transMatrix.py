# -*- coding: utf-8 -*-
import csv
import cv2
import numpy as np
from tf.transformations import quaternion_from_matrix, euler_from_matrix, quaternion_from_euler, quaternion_matrix
import math
from ar_func import read_csv2

def degrees_to_radians(degs):
    return np.array([ d / 180 * math.pi for d in degs])

def radian_to_quaternion(rad):
    return quaternion_from_euler(rad[0], rad[1], rad[2])

def radians_to_quaternions(rads):
    return np.array([radian_to_quaternion(r) for r in rads])
def col_csv(col):
    buf = ""
    for n in col:
        buf = buf + str(n) + " ,"
    buf = buf[:-2]
    buf = buf + "\n"
    return buf
def array1_csv(arr):
    buf = ""
    for n in arr:
        buf = buf + str(n) + " ,"
    buf = buf[:-2]
    return buf    
def mat_csv(mat):
    buf = ""
    for col in mat:
        buf = buf + col_csv(col)
    return buf

def get_translate_matrix(r_poses, ar_poses):
    y = r_poses[:, :3]
    y1 = y[1, :] - y[0, :]
    y2 = y[2, :] - y[0, :]
    y3 = np.cross(y2,y1)
    Y = np.array([y1,y2,y3]).T
    print("Y: ", Y)

    x = ar_poses[:, :3]
    x1 = x[1, :] - x[0, :]
    x2 = x[2, :] - x[0, :]
    x3 = np.cross(x2,x1)
    X = np.array([x1,x2,x3]).T
    print("X: ", X)

    A = np.dot(Y, np.linalg.inv(X))
    print("A: ", A)
    T1 = y[0, :] - np.dot(A, x[0, :])
    T2 = y[1, :] - np.dot(A, x[1, :])
    T3 = y[2, :] - np.dot(A, x[2, :])
    print("T: ", T1, T2, T3)
    Translate_CamM = RmatTvec_to_cameraMatrix(A, T1)
    print("Translate Camera Matrix: ", Translate_CamM)
    return Translate_CamM
def vector_to_quarernion(vector):
    R = vector_to_matrix(vector)
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((np.zeros(R.shape[1]), np.ones(1)))[np.newaxis, :].T))
    Q = quaternion_from_matrix(C)
    return Q
def matrix_to_vector(matrix):
    return vector_to_matrix(matrix)
def vector_to_matrix(vector):
    R = cv2.Rodrigues(vector)[0]
    return R
def RmatTvec_to_cameraMatrix(R,T):
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((T, np.ones(1)))[np.newaxis, :].T))
    return C
def quaternion_to_vector(quaternion):
    R = quaternion_matrix(quaternion)[:3,:3]
    V = cv2.Rodrigues(R)[0]
    return V

def estimate(T_camM, input):
    T = input[:3].astype(np.float32)
    Rvec = quaternion_to_vector(input[3:].astype(np.float32))
    R = vector_to_matrix(Rvec)
    C = RmatTvec_to_cameraMatrix(R, T)
    Est_C = np.dot(T_camM, C)
    Est_pos = Est_C[:3, 3]
    Est_R = Est_C[:3, :3]
    Est_vec = matrix_to_vector(Est_R)
    Est_Quat = vector_to_quarernion(Est_vec)
    return Est_Quat, Est_pos

def title_pose(str):
    buf = str + "_x" + " ,"
    buf = buf + str + "_y" + " ,"
    buf = buf + str + "_z" + " ,"
    buf = buf + str + "_qx" + " ,"
    buf = buf + str + "_qy" + " ,"
    buf = buf + str + "_qz" + " ,"
    buf = buf + str + "_qw" + " ,"
    return buf

def title_diff(str1, str2):
    buf = str1 + "-" + str2 + "_x" + " ,"
    buf = buf + str1 + "-" + str2 + "_y" + " ,"
    buf = buf + str1 + "-" + str2 + "_z" + " ,"
    buf = buf + str1 + "-" + str2 + "_roll" + " ,"
    buf = buf + str1 + "-" + str2 + "_pitch" + " ,"
    buf = buf + str1 + "-" + str2 + "_yaw" + " ,"
    return buf

def title():
    buf = ""
    buf = buf + title_pose("floor")
    buf = buf + title_pose("camera")
    buf = buf + title_pose("est_cam")
    buf = buf + title_pose("robot")
    buf = buf + title_pose("est_robot")
    buf = buf + title_diff("est_cam", "floor")
    buf = buf + title_diff("est_robot", "floor")
    buf = buf + title_diff("robot", "floor")
    buf = buf + title_diff("est_cam", "robot")
    buf = buf[:-2]
    buf = buf + "\n"
    return buf

def title_ar():
    buf = ""
    buf = buf + title_pose("floor")
    buf = buf + title_pose("camera")
    buf = buf + title_pose("est_cam")
    buf = buf + title_diff("est_cam", "floor")
    buf = buf[:-2]
    buf = buf + "\n"
    return buf

def title_rb():
    buf = ""
    buf = buf + title_pose("floor")
    buf = buf + title_pose("robot")
    buf = buf + title_pose("est_rb")
    buf = buf + title_diff("est_rb", "floor")
    buf = buf[:-2]
    buf = buf + "\n"
    return buf

def title_difference():
    buf = ""
    buf = buf + title_pose("floor")
    buf = buf + title_pose("est_cam")
    buf = buf + title_pose("est_rb")
    buf = buf + title_diff("est_cam", "est_rb")
    buf = buf[:-2]
    buf = buf + "\n"
    return buf

def main():
    path = '/home/minipc1/camera_ws/src/camera-controller/config/log_image_robot/Pose_2020-12-11_162050/'
    #source_list = [1, 4, 6]
    #source_list = [0, 2, 4]
    #source_list = [14, 20, 24]
    source_list = [0, 1, 2]#[12, 16, 22]#
    
    ar_path = path + 'ar.csv'
    floor_path = path + 'floor.csv'
    rb_path = path + 'pose.csv'
    result_path = path + 'result_deference.csv'
    result_ar = path + 'result_ar.csv'
    result_rb = path + 'result_rb.csv'
    result_dif = path + 'result_diff.csv'
    tmatrix_f_ar = path + 'tmatrix_f_ar.csv'
    tmatrix_f_rb = path + 'tmatrix_f_rb.csv'
    ar = read_csv2(ar_path)
    floor = read_csv2(floor_path)
    rb = read_csv2(rb_path)
    ar = ar[:,1:].astype(np.float32)
    ar_source = ar[source_list]
    floor = floor[:,1:].astype(np.float32)
    floor_source = floor[source_list]
    rb = rb[:,1:].astype(np.float32)
    rb_source = rb[source_list]


    tmatrix_ar_floor = get_translate_matrix(floor_source, ar_source)
    buf = mat_csv(tmatrix_ar_floor)
    with open(tmatrix_f_ar, mode='w') as f:
        f.write(buf)
    tmatrix_rb_floor = get_translate_matrix(floor_source, rb_source)
    buf = mat_csv(tmatrix_rb_floor)
    with open(tmatrix_f_rb, mode='w') as f:
        f.write(buf)

    ar_buf = title_ar()
    rb_buf = title_rb()
    diff_buf = title_difference()
    print(diff_buf)
    buf = title()
    for (floor_pose, ar_pose, rb_pose) in zip(floor, ar, rb):
        floor_R = quaternion_matrix(floor_pose[3:].astype(np.float32))[:3,:3]

        est_ar_quat, est_ar_pos = estimate(tmatrix_ar_floor, ar_pose)
        diff_est_ar_floor = est_ar_pos - floor_pose[:3].astype(np.float32)
        est_ar_R = quaternion_matrix(est_ar_quat)[:3,:3]
        diff_R_est_ar_floor = np.dot(np.linalg.inv(floor_R), est_ar_R)
        diff_rad_est_ar_floor = euler_from_matrix(diff_R_est_ar_floor)
        ar_buf = ar_buf + array1_csv(floor_pose) + " ,"
        ar_buf = ar_buf + array1_csv(ar_pose) + " ,"
        ar_buf = ar_buf + array1_csv(est_ar_pos) + " ,"
        ar_buf = ar_buf + array1_csv(est_ar_quat) + " ,"
        ar_buf = ar_buf + array1_csv(diff_est_ar_floor) + " ,"
        ar_buf = ar_buf + array1_csv(diff_rad_est_ar_floor) + "\n" 

        est_rb_quat, est_rb_pos = estimate(tmatrix_rb_floor, rb_pose)
        diff_est_rb_floor = est_rb_pos - floor_pose[:3].astype(np.float32)
        est_rb_R = quaternion_matrix(est_rb_quat)[:3,:3]
        diff_R_est_rb_floor = np.dot(np.linalg.inv(floor_R), est_rb_R)
        diff_rad_est_rb_floor = euler_from_matrix(diff_R_est_rb_floor)

        rb_buf = rb_buf + array1_csv(floor_pose) + " ,"
        rb_buf = rb_buf + array1_csv(rb_pose) + " ,"
        rb_buf = rb_buf + array1_csv(est_rb_pos) + " ,"
        rb_buf = rb_buf + array1_csv(est_rb_quat) + " ,"
        rb_buf = rb_buf + array1_csv(diff_est_rb_floor) + " ,"
        rb_buf = rb_buf + array1_csv(diff_rad_est_rb_floor) + "\n" 

        robot_R = quaternion_matrix(rb_pose[3:].astype(np.float32))[:3,:3]
        diff_est_ar_robot = est_ar_pos - rb_pose[:3].astype(np.float32)
        diff_robot_floor = rb_pose[:3].astype(np.float32) - floor_pose[:3].astype(np.float32)
        diff_R_est_ar_robot = np.dot(np.linalg.inv(robot_R), est_ar_R)
        diff_R_robot_floor = np.dot(np.linalg.inv(floor_R), robot_R)
        diff_rad_est_ar_robot = euler_from_matrix(diff_R_est_ar_robot)
        diff_rad_robot_floor = euler_from_matrix(diff_R_robot_floor)
        buf = buf + array1_csv(floor_pose) + " ,"
        buf = buf + array1_csv(ar_pose) + " ,"
        buf = buf + array1_csv(est_ar_pos) + " ,"
        buf = buf + array1_csv(est_ar_quat) + " ,"
        buf = buf + array1_csv(rb_pose) + " ,"
        buf = buf + array1_csv(est_rb_pos) + " ,"
        buf = buf + array1_csv(est_rb_quat) + " ,"
        buf = buf + array1_csv(diff_est_ar_floor) + " ,"
        buf = buf + array1_csv(diff_rad_est_ar_floor) + " ,"
        buf = buf + array1_csv(diff_est_rb_floor) + " ,"
        buf = buf + array1_csv(diff_rad_est_rb_floor) + " ,"
        buf = buf + array1_csv(diff_robot_floor) + " ,"
        buf = buf + array1_csv(diff_rad_robot_floor) + " ,"
        buf = buf + array1_csv(diff_est_ar_robot) + " ,"
        buf = buf + array1_csv(diff_rad_est_ar_robot) + "\n" 

        diff_est_ar_est_rb = est_ar_pos - est_rb_pos
        diff_R_est_ar_est_rb = np.dot(np.linalg.inv(est_rb_R), est_ar_R)
        diff_rad_est_ar_est_rb = euler_from_matrix(diff_R_est_ar_est_rb)
        diff_buf = diff_buf + array1_csv(floor_pose) + " ,"
        diff_buf = diff_buf + array1_csv(est_ar_pos) + " ,"
        diff_buf = diff_buf + array1_csv(est_ar_quat) + " ,"
        diff_buf = diff_buf + array1_csv(est_rb_pos) + " ,"
        diff_buf = diff_buf + array1_csv(est_rb_quat) + " ,"
        diff_buf = diff_buf + array1_csv(diff_est_ar_est_rb) + " ,"
        diff_buf = diff_buf + array1_csv(diff_rad_est_ar_est_rb) + "\n" 

    with open(result_ar, mode='w') as f:
        f.write(ar_buf)
    with open(result_rb, mode='w') as f:
        f.write(rb_buf)
    with open(result_path, mode='w') as f:
        f.write(buf)
    with open(result_dif, mode='w') as f:
        f.write(diff_buf)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
