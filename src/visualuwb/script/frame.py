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
from geometry_msgs.msg import PointStamped
import rospy
import tf2_ros
import geometry_msgs.msg
import math
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from numpy import radians as deg2rad
from numpy import degrees as rad2deg
#from tf2_geometry.msg import transform_to_kdl

if __name__ == '__main__':
    rospy.init_node('uav_tf')
    
    #br  = tf2.TransformBroadcaster()
    br = tf.TransformBroadcaster()
    ls = tf.TransformListener()
        
    ned_tran = (0,0,0)
    ned_quar = quaternion_from_euler(0 ,pi,deg2rad(143-90))

    uwb_tran = (0,0,0)
    uwb_quar = quaternion_from_euler(0,0,deg2rad(143))

    rate = rospy.Rate(30.0)

    
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        
        br.sendTransform(ned_tran,ned_quar,now,"ned","vicon")
        br.sendTransform(uwb_tran,uwb_quar,now,"uwb","ned")  
 
        rate.sleep()
        

        