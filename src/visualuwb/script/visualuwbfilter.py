#!/usr/bin/env python
__author__ = 'Jeffsan'
from VU_filter import *
from sslib import *


uwb = UWBLocation() 
imu = IMULocation() 
global xe,q,a,r,icount

def statecallback(msg):
    global icount
    global xe
   
    pos        = array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
    
    anchor_pos = anchor[icount%4]
    icount     = icount+1
    y          = linalg.norm(pos - anchor_pos)+ random.randn(1,1)[0]*0.1

    (xe, p) = uwb.locate(xe, Q, 1.0/(100), y, anchor_pos,q, a, r)
    
    br  = tf.TransformBroadcaster()
    uwb_tran = xe[0:3]
    uwb_q    = xe[3:7]
    br.sendTransform(pos, q, msg.header.stamp, "uwb", "world")  
    #print state_estimation[0:3],pos


def imucallback(msg):
    global xe
    global q,a,r
    q  =  array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    r  =  array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.x])
    a  =  array([msg.linear_acceleration.x,msg.linear_acceleration.y, msg.linear_acceleration.z])
    #(state_estimation, p)=imu.locate(state_estimation, Q, 1.0/(100/1), euler, acc, rate)
    #br = tf.TransformBroadcaster()
    #br.sendTransform(uwb_tran,(0,0,0,1), rospy.Time.now(), "uwb","world")  



if __name__ == '__main__':

    rospy.init_node('uav_filter')
    rate = rospy.Rate(30.0)
    global xe,q,a,r,icount
    icount =0
    xe = zeros((1,11))[0]
    xe[6] = 1
    q = array([0,0,0,1])
    a = array([0,0,0])
    rospy.Subscriber("/ground_truth/state", Odometry, statecallback, queue_size = 1, buff_size= 1 )
    rospy.Subscriber("/raw_imu", Imu, imucallback, queue_size = 1, buff_size= 1)
    rospy.spin()
        
 