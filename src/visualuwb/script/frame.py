#!/usr/bin/env python
from matplotlib.lines import lineStyles
from numpy import *
from pykalman import KalmanFilter
from pykalman import UnscentedKalmanFilter,AdditiveUnscentedKalmanFilter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import rospy
import tf
import tf2_ros as tf2
from tf2 import tf2_kdl
from geometry_msgs.msg import PointStamped
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from tf import transformPoint
from numpy import radians as deg2rad
from numpy import degrees as rad2deg

if __name__ == '__main__':
    rospy.init_node('uav_tf')
    
    br  = tf2.TransformBroadcaster()
        
    ned_tf = geometry_msgs.msg.TransformStamped()
    ned_tf.header.frame_id = "vicon"
    ned_tf.child_frame_id = "ned"
    ned_tf.transform.translation.x = 0
    ned_tf.transform.translation.y = 0
    ned_tf.transform.translation.z = 0.0
    ned_q = quaternion_from_euler(0 ,pi,deg2rad(143-90))
    ned_tf.transform.rotation.x = ned_q[0]
    ned_tf.transform.rotation.y = ned_q[1]
    ned_tf.transform.rotation.z = ned_q[2]
    ned_tf.transform.rotation.w = ned_q[3]
    
    uwb_tf = geometry_msgs.msg.TransformStamped()
    uwb_tf.header.frame_id = "ned"
    uwb_tf.child_frame_id = "uwb"
    uwb_tf.transform.translation.x = 0
    uwb_tf.transform.translation.y = 0
    uwb_tf.transform.translation.z = 0.0
    uwb_q = quaternion_from_euler(0,0,deg2rad(143))
    uwb_tf.transform.rotation.x = uwb_q[0]
    uwb_tf.transform.rotation.y = uwb_q[1]
    uwb_tf.transform.rotation.z = uwb_q[2]
    uwb_tf.transform.rotation.w = uwb_q[3]
    
    body_tf = geometry_msgs.msg.TransformStamped()
    body_tf.header.frame_id = "ned"
    body_tf.child_frame_id  = "body"
    
    cam_tf = geometry_msgs.msg.TransformStamped()
    cam_tf.header.frame_id = "body"
    cam_tf.child_frame_id  = "cam"
    
    def do_transform_point(point, transform):
        p = transform_to_kdl(transform) * PyKDL.Vector(point.point.x, point.point.y, point.point.z)
        res = PointStamped()
        res.point.x = p[0]
        res.point.y = p[1]
        res.point.z = p[2]
        res.header = transform.header
        return res
    
    tf2_ros.TransformRegistration().add(PointStamped, do_transform_point)
    

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        
        uwb_tf.header.stamp = rospy.Time.now()  
        br.sendTransform(uwb_tf)
        
        ned_tf.header.stamp = rospy.Time.now()  
        br.sendTransform(ned_tf)

        
#===============================================================================
#         x = rospy.Time.now().to_sec() * math.pi 
#         body_tf.header.stamp = rospy.Time.now()   
#         body_tf.transform.translation.x = 1 * math.sin(x)
#         body_tf.transform.translation.y = 1 * math.cos(x)
#         body_tf.transform.translation.z = 3.0
#         body_tf.transform.rotation.x = 0.0
#         body_tf.transform.rotation.y = 0.0
#         body_tf.transform.rotation.z = 0.0
#         body_tf.transform.rotation.w = 1.0
#         br.sendTransform(body_tf)
#         
#         
#         uwb_tf.header.stamp = rospy.Time.now()   
#         uwb_tf.transform.translation.x = 0.5
#         uwb_tf.transform.translation.y = 0.5
#         uwb_tf.transform.translation.z = 0.5
#         uwb_tf.transform.rotation.x = 0.0
#         uwb_tf.transform.rotation.y = 0.0
#         uwb_tf.transform.rotation.z = 0.0
#         uwb_tf.transform.rotation.w = 1.0
#         br.sendTransform(uwb_tf)
# 
#         
#         cam_tf.header.stamp = rospy.Time.now()   
#         cam_tf.transform.translation.x = -0.5
#         cam_tf.transform.translation.y = 0.5 
#         cam_tf.transform.translation.z = 0.0
#         cam_tf.transform.rotation.x = 0.3
#         cam_tf.transform.rotation.y = 0.5
#         cam_tf.transform.rotation.z = 0.0
#         cam_tf.transform.rotation.w = 0.4
#         br.sendTransform(cam_tf)
#===============================================================================
        rate.sleep()
        

        
