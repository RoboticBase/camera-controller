# -*- coding: utf-8 -*-
import numpy as np
import cv2
from tf.transformations import decompose_matrix, euler_matrix, quaternion_from_matrix, quaternion_from_euler, quaternion_matrix, compose_matrix

def RMatRadToCMat(R,T):
    C = np.vstack((R, np.zeros(R.shape[1])))
    C = np.hstack((C, np.hstack((T, np.ones(1)))[np.newaxis, :].T))
    return C

def VectorToRMat(V):
    R = cv2.Rodrigues(np.squeeze(V))[0]
    return R

def VectorToCMat(V):
    C = VectorToRMat(V)
    C = np.vstack((C, np.zeros(C.shape[1])))
    C = np.hstack((C, np.hstack([0,0,0,1])[np.newaxis, :].T))
    return C

def CMatToQuat(C):
    return quaternion_from_matrix(C)

def CMatToRMat(R4):
    return np.delete(np.delete(R4, 2, 1), 2, 0)

def VectorToQuat(V):
    C = VectorToCMat(V)
    return CMatToQuat(C)

def RadianToMatrix(rads):
    return euler_matrix(rads[0], rads[1], rads[2], 'syxz')

def RadianToQuat(rad):
    return quaternion_from_euler(rad[0], rad[1], rad[2])

def RadiansToQuats(rads):
    return np.array([RadianToQuat(r) for r in rads])

def DegsToRadians(degs):
    return np.array([ d / 180 * np.pi for d in degs])

def DegsToQuats(degs):
    return RadiansToQuats(DegsToRadians(degs))

def QuatToCMat(Q):
    return quaternion_matrix(Q)

def QuatToVector(Q):
    R = QuatToCMat(Q)[:3,:3]
    V = RmatToVector(R)
    return V

def RmatToVector(R):
    return VectorToRMat(R)

def RmatToQuat(R):
    V = RmatToVector(R)
    return VectorToQuat(V)

def SSATPToCMat(scale, shear, angles, trans, persp):
    return compose_matrix(scale, shear, angles, trans, persp)

def CMatToSSATP(C):
    return decompose_matrix(C)

def Tmatrix(s_coord, o_coord):
    y = o_coord.loc[:,['x','y','z']]
    x = s_coord.loc[:,['x','y','z']]

    y1 = y.iloc[1].values - y.iloc[0].values
    y2 = y.iloc[2].values - y.iloc[0].values
    y3 = np.cross(y2,y1)
    Y = np.array([y1,y2,y3]).T

    x1 = x.iloc[1].values - x.iloc[0].values
    x2 = x.iloc[2].values - x.iloc[0].values
    x3 = np.cross(x2,x1)
    X = np.array([x1,x2,x3]).T

    A = np.dot(Y, np.linalg.inv(X))
    T1 = y.iloc[0].values - np.dot(A, x.iloc[0].values)
    T2 = y.iloc[1].values - np.dot(A, x.iloc[1].values)
    T3 = y.iloc[2].values - np.dot(A, x.iloc[2].values)
    Translate_CamM = RMatRadToCMat(A, T1)

    return Translate_CamM

def translate(Tmat, input):
    T = input[:3].astype(np.float32)
    Rvec = QuatToVector(input[3:].astype(np.float32))
    R = VectorToRMat(Rvec)
    C = RMatRadToCMat(R, T)
    estC = np.dot(Tmat, C)
    return estC

def translateCoordinatesC(Tmat, input):
    return translate(Tmat, input)

def translateCoordinatesQT(Tmat, input):
    estC = translate(Tmat, input)
    estT = estC[:3, 3]
    estR = estC[:3, :3]
    estQuat = RmatToQuat(estR)
    return estQuat, estT
