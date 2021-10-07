# -*- coding: utf-8 -*-
import numpy as np
import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, PoseWithCovarianceStamped

from rotation import VectorToQuat, RadianToQuat

def PoseStampedToOrgarray(msg):
    T, Q = PoseStampedToTQ(msg)
    rads = euler_from_quaternion(Q)
    return T.tolist() + list(rads)

def PoseStampedToTQ(msg):
    Tvec = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype = 'float')
    Quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat

def PoseStampedToNumpyarray(msg):
    Tvec, Quat = PoseStampedToTQ(msg)
    TQ = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, 
                msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w], dtype = 'float')
    return Tvec, Quat, TQ

def TQToPose(T, Q):
    p = Pose()
    p.position.x = T[0]
    p.position.y = T[1]
    p.position.z = T[2]
    p.orientation.x = Q[0]
    p.orientation.y = Q[1]
    p.orientation.z = Q[2]
    p.orientation.w = Q[3]
    return p

def MeanToPose(mean):
    return TQToPose(mean[:3], RadianToQuat(mean[3:]))

def MeanToPoseStamped(mean, frame_id):
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    p.pose = MeanToPose(mean)
    return p    

def VectorsToPoseStamped(rvecs, tvecs, frame_id):
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.header.stamp = rospy.Time.now()
    q = VectorToQuat(rvecs)
    tvecs = np.squeeze(tvecs)
    p.pose = TQToPose(tvecs, q)
    return p

def MeanCovToPoseWithCovariance(mean, cov):
    pc = PoseWithCovariance()
    pc.pose = TQToPose(mean[:3], RadianToQuat(mean[3:]))
    pc.covariance = cov.ravel().tolist()
    return pc

def MeanCovToPoseWithCovarianceStamped(mean, cov, frame_id):
    pcs = PoseWithCovarianceStamped()
    pcs.header.frame_id = frame_id
    pcs.header.stamp = rospy.Time.now()
    pcs.pose = MeanCovToPoseWithCovariance(mean, cov)
    return pcs