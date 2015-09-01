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
import geometry_msgs.msg
import turtlesim.msg
import rospy
import tf2_ros
import geometry_msgs.msg
import math

if __name__ == '__main__':
    rospy.init_node('uav_tf')
    
    br  = tf2.TransformBroadcaster()
    #body_br = tf2.TransformBroadcaster()
    #uwb_br  = tf2.TransformBroadcaster()
    #cam_br  = tf2.TransformBroadcaster()
    
    map_tf = geometry_msgs.msg.TransformStamped()
    map_tf.header.frame_id = "ned"
    map_tf.child_frame_id = "map"
    
    body_tf = geometry_msgs.msg.TransformStamped()
    body_tf.header.frame_id = "map"
    body_tf.child_frame_id = "body"
    
    uwb_tf = geometry_msgs.msg.TransformStamped()
    uwb_tf.header.frame_id = "body"
    uwb_tf.child_frame_id = "uwb"
    
    cam_tf = geometry_msgs.msg.TransformStamped()
    cam_tf.header.frame_id = "body"
    cam_tf.child_frame_id = "cam"
    
    
    map_tf.transform.translation.x = 1
    map_tf.transform.translation.y = 1
    map_tf.transform.translation.z = 0.0
    map_tf.transform.rotation.x = 0.0
    map_tf.transform.rotation.y = 0.0
    map_tf.transform.rotation.z = 0.0
    map_tf.transform.rotation.w = 1.0

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        
        map_tf.header.stamp = rospy.Time.now()  
        br.sendTransform(map_tf)
        #rate.sleep()
        
        x = rospy.Time.now().to_sec() * math.pi 
        body_tf.header.stamp = rospy.Time.now()   
        body_tf.transform.translation.x = 1 * math.sin(x)
        body_tf.transform.translation.y = 1 * math.cos(x)
        body_tf.transform.translation.z = 1.0
        body_tf.transform.rotation.x = 0.0
        body_tf.transform.rotation.y = 0.0
        body_tf.transform.rotation.z = 0.0
        body_tf.transform.rotation.w = 1.0
        br.sendTransform(body_tf)
        #rate.sleep()
        
        
        uwb_tf.header.stamp = rospy.Time.now()   
        uwb_tf.transform.translation.x = 0.1
        uwb_tf.transform.translation.y = 0.1
        uwb_tf.transform.translation.z = 0.1
        uwb_tf.transform.rotation.x = 0.0
        uwb_tf.transform.rotation.y = 0.0
        uwb_tf.transform.rotation.z = 0.0
        uwb_tf.transform.rotation.w = 1.0
        br.sendTransform(uwb_tf)
        #rate.sleep()
        
        cam_tf.header.stamp = rospy.Time.now()   
        cam_tf.transform.translation.x = -0.1
        cam_tf.transform.translation.y = 0.1 
        cam_tf.transform.translation.z = 0.0
        cam_tf.transform.rotation.x = 0.3
        cam_tf.transform.rotation.y = 0.5
        cam_tf.transform.rotation.z = 0.0
        cam_tf.transform.rotation.w = 0.4
        br.sendTransform(cam_tf)
        rate.sleep()
        

        
