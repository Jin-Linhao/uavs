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
import rospy
import geometry_msgs.msg
import math
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':
    rospy.init_node('require_tf')

    #tfBuffer = tf.Buffer()
    #listener = tf.TransformListener(tfBuffer)
    #listener.transformPoint()#(target_frame_, *point_ptr, point_out);

    rate = rospy.Rate(30.0)
    p = PointStamped()
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            #(trans,rot) = listener.lookupTransform('ned', '/uwb', rospy.Time(0))   
            now = rospy.Time.now()
            listener.waitForTransform("ned", "uwb", now, rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("ned", "uwb", now)
            print trans,rot
        except (tf2.LookupException, tf2.ConnectivityException, tf2.ExtrapolationException):
            rate.sleep()
            continue

        rate.sleep()