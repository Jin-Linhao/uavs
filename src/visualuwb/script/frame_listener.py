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
    rospy.init_node('require_tf')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            ned_cam_tf = tfBuffer.lookup_transform('ned', 'cam', rospy.Time.now(),rospy.Duration(0.5))
            print ned_cam_tf.transform.quaternion_from_euler()
            print ned_cam_tf
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        rate.sleep()