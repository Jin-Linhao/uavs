#!/usr/bin/env python
__author__ = 'Jeffsan'
from VU_filter import *
from sslib import *


Q[ 0:3,  0:3] =  1.28*eye(3)#*10
Q[ 3:7,  3:7] =  0.01*eye(4)#*10
Q[ 7:9,  7:9] =  0.81*eye(2)#/2
Q[  9 ,   9 ] =  400#/2
Q[ 10 ,  10 ] =  0.000000001
uwb = UWBLocation(0.01) 
uwb.setQ(Q)   
vision = VisionlLocation(1.0/100, K) 
vision.setQ(Q)
global xe,q,a,r,icount

def statecallback(msg):
    global icount
    global xe


   
    pos        = array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    anchor_pos = uwbanchor[icount%4]
    icount     = icount+1
    y          = linalg.norm(pos - anchor_pos)+ random.randn(1,1)[0]*0.1
    
    visionmeasure = array([ dot(K, pos - visionanchor[i]) for i in xrange(4)]).reshape((8))+ random.randn(8)
    
    if icount % 2 == 0:
        xe, _ = uwb.locate(xe, Q, 1.0/(100), y, anchor_pos,q, a, r)
        #print "uwb"
    else:
        xe, _ = vision.locate(xe, Q, 1.0/100, visionmeasure, visionanchor, q, a, r)
        #print 'vision'
   
    
    br  = tf.TransformBroadcaster()
    uwb_tran = xe[0:3]
    uwb_q    = xe[3:7]
    br.sendTransform(uwb_tran, uwb_q, msg.header.stamp, "VisionUWB", "world")  
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
    xe[2] = 0.27
    q = array([0,0,0,1])
    a = array([0,0,0])
    r = array([0,0,0])
    rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped, statecallback, queue_size = 1, buff_size= 1,tcp_nodelay=True )
    rospy.Subscriber("/raw_imu", Imu, imucallback, queue_size = 1, buff_size= 1,tcp_nodelay=True)
    rospy.spin()
        
 