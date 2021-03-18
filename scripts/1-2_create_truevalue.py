# -*- coding: utf-8 -*-
import csv
import numpy as np
from tf.transformations import quaternion_from_euler
import tf
import math

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
def mat_csv(mat):
    buf = ""
    for col in mat:
        buf = buf + col_csv(col)
    return buf
def main():
    output = '/home/minipc1/camera_ws/src/camera-controller/config/log_image_robot/Pose_2020-12-11_162050/floor.csv'
    num = np.array(range(3)).reshape(-1, 1)
    point0 = [ 0, -1,0,0,0,  0]
    point1 = [ 1, 1,0,0,0, 90]
    point2 = [-1, 1,0,0,0, 180]
    true_val = np.array([point0, point1, point2])
    '''
    point0 = [0,0,0,0,0,0]
    point1 = [0.5,0,0,0,0,0]
    point2 = [0.5,0.5,0,0,0,90]
    point3 = [0,0.5,0,0,0,180]
    point4 = [-0.5,0.5,0,0,0,180]
    point5 = [-0.5,0,0,0,0,-90]
    point6 = [-0.5,-0.5,0,0,0,-90]
    point7 = [0,-0.5,0,0,0,0]
    point8 = [0.5,-0.5,0,0,0,0]
    point9 = [1.0,-0.5,0,0,0,0]
    point10 = [1.0,0,0,0,0,90]
    point11 = [1.0,0.5,0,0,0,90]
    point12 = [1.0,1.0,0,0,0,90]
    point13 = [0.5,1.0,0,0,0,180]
    point14 = [0,1.0,0,0,0,180]
    point15 = [-0.5,1.0,0,0,0,180]
    point16 = [-1.0,1.0,0,0,0,180]
    point17 = [-1.0,0.5,0,0,0,-90]
    point18 = [-1.0,0,0,0,0,-90]
    point19 = [-1.0,-0.5,0,0,0,-90]
    point20 = [-1.0,-1.0,0,0,0,-90]
    point21 = [-0.5,-1.0,0,0,0,0]
    point22 = [0,-1.0,0,0,0,0]
    point23 = [0.5,-1.0,0,0,0,0]
    point24 = [1.0,-1.0,0,0,0,0]
    point25 = [1.5,-1.0,0,0,0,0]
    point26 = [1.5,-0.5,0,0,0,90]
    point27 = [1.5,0,0,0,0,90]
    point28 = [1.5,0.5,0,0,0,90]
    point29 = [1.5,1.0,0,0,0,90]
    point30 = [1.5,1.5,0,0,0,90]
    point31 = [1.0,1.5,0,0,0,180]
    point32 = [0.5,1.5,0,0,0,180]
    point33 = [0,1.5,0,0,0,180]
    point34 = [-0.5,1.5,0,0,0,180]
    point35 = [-1.0,1.5,0,0,0,180]
    point36 = [-1.5,1.5,0,0,0,180]
    point37 = [-1.5,1.0,0,0,0,-90]
    point38 = [-1.5,0.5,0,0,0,-90]
    point39 = [-1.5,0,0,0,0,-90]
    point40 = [-1.5,-0.5,0,0,0,-90]
    point41 = [-1.5,-1.0,0,0,0,-90]
    point42 = [-1.5,-1.5,0,0,0,-90]
    point43 = [-1.0,-1.5,0,0,0,0]
    point44 = [-0.5,-1.5,0,0,0,0]
    point45 = [0,-1.5,0,0,0,0]
    point46 = [0.5,-1.5,0,0,0,0]
    point47 = [1.0,-1.5,0,0,0,0]
    point48 = [1.5,-1.5,0,0,0,90]

    point0 = [ 0.5,   0,0,0,0,  0]
    point1 = [-0.5,-0.5,0,0,0, 180]
    point2 = [-0.5, 0.5,0,0,0,  0]
    true_val = np.array([point0, point1, point2])

    point0 = [1,-1,0,0,0,0]
    point1 = [0,1,0,0,0,90]
    point2 = [-1,-1,0,0,0,180]
    true_val = np.array([point0, point1, point2])

    point0 = [0,0,0,0,0,0]
    point1 = [0.5,0,0,0,0,0]
    point2 = [0.5,0.5,0,0,0,90]
    point3 = [0,0.5,0,0,0,180]
    point4 = [-0.5,0.5,0,0,0,180]
    point5 = [-0.5,0,0,0,0,-90]
    point6 = [-0.5,-0.5,0,0,0,-90]
    point7 = [0,-0.5,0,0,0,0]
    point8 = [0.5,-0.5,0,0,0,0]
    point9 = [1.0,-0.5,0,0,0,0]
    point10 = [1.0,0,0,0,0,90]
    point11 = [1.0,0.5,0,0,0,90]
    point12 = [1.0,1.0,0,0,0,90]
    point13 = [0.5,1.0,0,0,0,180]
    point14 = [0,1.0,0,0,0,180]
    point15 = [-0.5,1.0,0,0,0,180]
    point16 = [-1.0,1.0,0,0,0,180]
    point17 = [-1.0,0.5,0,0,0,-90]
    point18 = [-1.0,0,0,0,0,-90]
    point19 = [-1.0,-0.5,0,0,0,-90]
    point20 = [-1.0,-1.0,0,0,0,-90]
    point21 = [-0.5,-1.0,0,0,0,0]
    point22 = [0,-1.0,0,0,0,0]
    point23 = [0.5,-1.0,0,0,0,0]
    point24 = [1.0,-1.0,0,0,0,0]
    point25 = [1.5,-1.0,0,0,0,0]
    point26 = [1.5,-0.5,0,0,0,90]
    point27 = [1.5,0,0,0,0,90]
    point28 = [1.5,0.5,0,0,0,90]
    point29 = [1.5,1.0,0,0,0,90]
    point30 = [1.5,1.5,0,0,0,90]
    point31 = [1.0,1.5,0,0,0,180]
    point32 = [0.5,1.5,0,0,0,180]
    point33 = [0,1.5,0,0,0,180]
    point34 = [-0.5,1.5,0,0,0,180]
    point35 = [-1.0,1.5,0,0,0,180]
    point36 = [-1.5,1.5,0,0,0,-90]
    point37 = [-1.5,1.0,0,0,0,-90]
    point38 = [-1.5,0.5,0,0,0,-90]
    point39 = [-1.5,0,0,0,0,-90]
    point40 = [-1.5,-0.5,0,0,0,-90]
    point41 = [-1.5,-1.0,0,0,0,-90]
    point42 = [-1.5,-1.5,0,0,0,-90]
    point43 = [-1.0,-1.5,0,0,0,0]
    point44 = [-0.5,-1.5,0,0,0,0]
    point45 = [0,-1.5,0,0,0,0]
    point46 = [0.5,-1.5,0,0,0,0]
    point47 = [1.0,-1.5,0,0,0,0]
    point48 = [1.5,-1.5,0,0,0,0]
    '''

    #true_val = np.array([point0, point1, point2, point3, point4, point5, point6, point7, point8, point9, point10, point11, point12, point13, point14, point15, point16, point17, point18, point19, point20, point21, point22, point23, point24, point25, point26, point27, point28, point29, point30, point31, point32, point33, point34, point35, point36, point37, point38, point39, point40, point41, point42, point43, point44, point45, point46, point47, point48])
    true_dir = true_val[:,0:3].astype(np.float32)
    true_ori = true_val[:,3:7].astype(np.float32)
    true_quat = radians_to_quaternions(degrees_to_radians(true_ori))
    C = np.hstack((num, true_dir))
    C = np.hstack((C, true_quat))
    print(C)
    buf = mat_csv(C)
    with open(output, mode='w') as f:
        f.write(buf)
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
