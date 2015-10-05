#!/usr/bin/env python
from sslib import *

if __name__ == '__main__':
    
    rospy.init_node('controller', anonymous=True)
    
    br = tf.TransformBroadcaster()
    
    edge = 1.0
    
    uav0_t = (0, edge, 0)
    uav0_q = quaternion_from_euler(0, 0, 0)
    
    uav1_t = ( edge * sqrt(3)/2, -edge/2, 0)
    uav1_q = quaternion_from_euler(0, 0, 0)
    
    uav2_t = (-edge * sqrt(3)/2, -edge/2, 0)
    uav2_q = quaternion_from_euler(0, 0, 0)
    
    
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        br.sendTransform(uav0_t, uav0_q, now, '/target/uav0','/target/base_stabilized')
        br.sendTransform(uav1_t, uav1_q, now, '/target/uav1','/target/base_stabilized')
        br.sendTransform(uav2_t, uav2_q, now, '/target/uav2','/target/base_stabilized')